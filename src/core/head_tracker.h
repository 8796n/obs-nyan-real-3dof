// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Head pose tracker: gyro integration with gravity correction (on_imu) or
// device-fused orientation passthrough (on_external_pose). Mirrors the
// headTracker flow of xrealonenet/3dof/js/app.js with the calibration
// extensions described in docs/architecture.md.
#pragma once

#include <cstdint>

#include "math_util.h"
#include "nyan_log.h"
#include "nyan_types.h"

constexpr double MAG_YAW_TAU_S = 8.0;
constexpr uint32_t MAG_STALE_US = 200000;
constexpr float MAG_SAT_ABS = 3199.5f;
constexpr double STATIONARY_ACCEL_REL_TOL = 0.18;
constexpr double STATIONARY_GYRO_TOL = 0.05;
constexpr double STATIONARY_CONFIRM_S = 0.5;
// Gyro-bias calibration guards (an intentional extension over the mirrored
// app.js flow). The warm-up skips sensor settling after stream start and the
// user's hand leaving the glasses after pressing recalibrate. The std
// tolerances reject averaging windows captured while the glasses move: a bias
// estimated from motion both drifts and blocks the stationary auto-adaptation
// (its threshold compares bias-corrected gyro readings).
constexpr double CALIBRATION_WARMUP_S = 0.5;
constexpr double CALIBRATION_GYRO_STD_TOL = 0.03;      // rad/s, per axis
constexpr double CALIBRATION_ACCEL_STD_REL_TOL = 0.05; // relative to |a|
constexpr double GYRO_BIAS_ADAPT_TAU_XY_S = 60.0;
constexpr double GYRO_BIAS_ADAPT_TAU_Z_S = 18.0;
constexpr double STATIONARY_DECAY_RATE = 2.0;
constexpr double GYRO_SCALE = 1.0;

class head_tracker {
public:
	void reset()
	{
		is_calibrated_ = false;
		restart_window();
		calibration_warmup_ts_us_ = 0;
		gyro_bias_ = {};
		accel_ref_norm_ = 1.0;
		last_imu_ts_us_ = 0;
		stationary_time_s_ = 0.0;
		bias_auto_active_ = false;
		q_ = {};
		zq_ = {};
		mag_yaw_corr_ = {};
		last_mag_ts_used_ = 0;
		mag_ref_valid_ = false;
		mag_heading_ref_rad_ = 0.0;
		mag_yaw_ref_rad_ = 0.0;
		latest_mag_ = {};
		latest_omega_ = {};
		imu_count_ = 0;
		mag_count_ = 0;
		update_mount();
	}

	void restart_calibration()
	{
		is_calibrated_ = false;
		restart_window();
		calibration_warmup_ts_us_ = 0;
		gyro_bias_ = {};
		last_imu_ts_us_ = 0;
		stationary_time_s_ = 0.0;
		bias_auto_active_ = false;
	}

	void set_debug(bool enabled) { debug_log_ = enabled; }

	void set_mount_deg(double deg)
	{
		mount_x_deg_ = deg;
		update_mount();
	}

	void set_mag_yaw_enabled(bool enabled)
	{
		if (mag_yaw_enabled_ == enabled)
			return;
		mag_yaw_enabled_ = enabled;
		reset_mag_yaw_correction();
	}

	void recenter()
	{
		zq_ = quat_inverse(q_);
		reset_mag_yaw_correction();
	}

	void on_mag(const mag_sample &m)
	{
		latest_mag_ = m;
		mag_count_++;
	}

	void on_imu(const imu_sample &raw)
	{
		const imu_sample imu = apply_mount(raw);
		imu_count_++;
		if (!is_calibrated_) {
			calibrate(imu);
			return;
		}
		update(imu);
	}

	// Devices that fuse orientation on-board (VITURE) bypass gyro
	// integration and calibration entirely: store the pose, apply the
	// mount tilt as a frame conjugation via right-multiplication, and
	// derive omega from successive poses for render-time prediction.
	void on_external_pose(const quatd &device_q, uint32_t ts_us)
	{
		const quatd q = quat_normalize(quat_multiply(device_q, mount_q_));
		if (is_calibrated_ && last_imu_ts_us_ != 0) {
			const double dt =
				elapsed_us32(ts_us, last_imu_ts_us_) / 1e6;
			if (dt > 1e-4 && dt < 0.2) {
				quatd dq = quat_multiply(quat_inverse(q_), q);
				const double s = (dq.w >= 0.0 ? 2.0 : -2.0) / dt;
				latest_omega_ = {s * dq.x, s * dq.y, s * dq.z};
			}
		}
		q_ = q;
		last_imu_ts_us_ = ts_us ? ts_us : 1;
		is_calibrated_ = true;
		imu_count_++;
	}

	pose_snapshot snapshot() const
	{
		quatd qrel = quat_multiply(zq_, q_);
		if (mag_yaw_enabled_)
			qrel = quat_multiply(mag_yaw_corr_, qrel);
		qrel = quat_normalize(qrel);
		pose_snapshot s = {qrel,  latest_omega_,     last_imu_ts_us_,
				   is_calibrated_, false, bias_auto_active_,
				   imu_count_,     mag_count_};
		return s;
	}

private:
	void update_mount()
	{
		mount_q_ = quat_from_rot_x(mount_x_deg_ * PI / 180.0);
	}

	void reset_mag_yaw_correction()
	{
		mag_yaw_corr_ = {};
		last_mag_ts_used_ = 0;
		mag_ref_valid_ = false;
		mag_heading_ref_rad_ = 0.0;
		mag_yaw_ref_rad_ = 0.0;
	}

	imu_sample apply_mount(const imu_sample &in) const
	{
		const quatd mq = quat_normalize(mount_q_);
		const vec3d g = rotate_vector(mq, {in.gx, in.gy, in.gz});
		const vec3d a = rotate_vector(mq, {in.ax, in.ay, in.az});
		imu_sample out = in;
		out.gx = static_cast<float>(g.x);
		out.gy = static_cast<float>(g.y);
		out.gz = static_cast<float>(g.z);
		out.ax = static_cast<float>(a.x);
		out.ay = static_cast<float>(a.y);
		out.az = static_cast<float>(a.z);
		return out;
	}

	mag_sample apply_mount_to_mag(const mag_sample &in) const
	{
		const quatd mq = quat_normalize(mount_q_);
		const vec3d m = rotate_vector(mq, {in.mx, in.my, in.mz});
		mag_sample out = in;
		out.mx = static_cast<float>(m.x);
		out.my = static_cast<float>(m.y);
		out.mz = static_cast<float>(m.z);
		return out;
	}

	void calibrate(const imu_sample &imu)
	{
		const uint32_t t = imu.ts_us;
		if (!calibration_warmup_ts_us_) {
			calibration_warmup_ts_us_ = t ? t : 1;
			return;
		}
		if (elapsed_us32(t, calibration_warmup_ts_us_) <
		    static_cast<uint32_t>(CALIBRATION_WARMUP_S * 1e6))
			return;

		if (!calibration_start_ts_us_)
			calibration_start_ts_us_ = t ? t : 1;
		sum_g_.x += imu.gx;
		sum_g_.y += imu.gy;
		sum_g_.z += imu.gz;
		sum_g2_.x += static_cast<double>(imu.gx) * imu.gx;
		sum_g2_.y += static_cast<double>(imu.gy) * imu.gy;
		sum_g2_.z += static_cast<double>(imu.gz) * imu.gz;
		sum_a_.x += imu.ax;
		sum_a_.y += imu.ay;
		sum_a_.z += imu.az;
		const double an = std::sqrt(static_cast<double>(imu.ax) * imu.ax +
					    static_cast<double>(imu.ay) * imu.ay +
					    static_cast<double>(imu.az) * imu.az);
		sum_an_ += an;
		sum_an2_ += an * an;
		calibration_count_++;

		const uint32_t elapsed = elapsed_us32(t, calibration_start_ts_us_);
		if (calibration_count_ < calibration_min_samples_ ||
		    elapsed < calibration_min_duration_us_)
			return;

		const double inv = 1.0 / static_cast<double>(calibration_count_);
		const vec3d mean_g = {sum_g_.x * inv, sum_g_.y * inv,
				      sum_g_.z * inv};
		const double var_gx =
			std::max(0.0, sum_g2_.x * inv - mean_g.x * mean_g.x);
		const double var_gy =
			std::max(0.0, sum_g2_.y * inv - mean_g.y * mean_g.y);
		const double var_gz =
			std::max(0.0, sum_g2_.z * inv - mean_g.z * mean_g.z);
		const double gyro_std =
			std::sqrt(std::max(var_gx, std::max(var_gy, var_gz)));
		const double mean_an = sum_an_ * inv;
		const double var_an =
			std::max(0.0, sum_an2_ * inv - mean_an * mean_an);
		const double accel_rel_std =
			mean_an > 1e-6 ? std::sqrt(var_an) / mean_an : 1.0;

		if (gyro_std > CALIBRATION_GYRO_STD_TOL ||
		    accel_rel_std > CALIBRATION_ACCEL_STD_REL_TOL) {
			if (debug_log_)
				nyan_log(NYAN_LOG_INFO,
				     "[obs-nyan-real-3dof] calibration window "
				     "rejected (gyro std %.4f rad/s, accel rel "
				     "std %.3f); retrying",
				     gyro_std, accel_rel_std);
			restart_window();
			return;
		}

		gyro_bias_ = mean_g;
		const vec3d avg_a = {sum_a_.x * inv, sum_a_.y * inv, sum_a_.z * inv};
		const double avg_an = std::sqrt(avg_a.x * avg_a.x +
						avg_a.y * avg_a.y +
						avg_a.z * avg_a.z);
		accel_ref_norm_ =
			(std::isfinite(avg_an) && avg_an > 1e-6) ? avg_an : 1.0;
		is_calibrated_ = true;
		last_imu_ts_us_ = 0;
		update_mount();
	}

	// Drop the current averaging window (keeps the warm-up state).
	void restart_window()
	{
		calibration_count_ = 0;
		calibration_start_ts_us_ = 0;
		sum_g_ = {};
		sum_g2_ = {};
		sum_a_ = {};
		sum_an_ = 0.0;
		sum_an2_ = 0.0;
	}

	void update(const imu_sample &imu)
	{
		const uint32_t t = imu.ts_us;
		if (!last_imu_ts_us_) {
			last_imu_ts_us_ = t;
			return;
		}

		const uint32_t d_us = elapsed_us32(t, last_imu_ts_us_);
		last_imu_ts_us_ = t;
		const double dt = static_cast<double>(d_us) / 1000000.0;
		if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.2)
			return;

		const double raw_wx = (imu.gx - gyro_bias_.x) * GYRO_SCALE;
		const double raw_wy = (imu.gy - gyro_bias_.y) * GYRO_SCALE;
		const double raw_wz = (imu.gz - gyro_bias_.z) * GYRO_SCALE;
		double wx = raw_wx;
		double wy = raw_wy;
		double wz = raw_wz;

		const double ax = imu.ax;
		const double ay = imu.ay;
		const double az = imu.az;
		const double a_norm = std::sqrt(ax * ax + ay * ay + az * az);
		const double gyro_norm =
			std::sqrt(raw_wx * raw_wx + raw_wy * raw_wy + raw_wz * raw_wz);
		const double accel_norm_err =
			std::abs(a_norm - accel_ref_norm_) / accel_ref_norm_;
		const bool is_stationary =
			std::isfinite(a_norm) && std::isfinite(accel_norm_err) &&
			accel_norm_err <= STATIONARY_ACCEL_REL_TOL &&
			gyro_norm <= STATIONARY_GYRO_TOL;

		if (is_stationary) {
			stationary_time_s_ = std::min(stationary_time_s_ + dt, 4.0);
			bias_auto_active_ = stationary_time_s_ >= STATIONARY_CONFIRM_S;
			if (bias_auto_active_) {
				const double k_xy =
					1.0 - std::exp(-dt / GYRO_BIAS_ADAPT_TAU_XY_S);
				const double k_z =
					1.0 - std::exp(-dt / GYRO_BIAS_ADAPT_TAU_Z_S);
				gyro_bias_.x += (imu.gx - gyro_bias_.x) * k_xy;
				gyro_bias_.y += (imu.gy - gyro_bias_.y) * k_xy;
				gyro_bias_.z += (imu.gz - gyro_bias_.z) * k_z;
				wx = (imu.gx - gyro_bias_.x) * GYRO_SCALE;
				wy = (imu.gy - gyro_bias_.y) * GYRO_SCALE;
				wz = (imu.gz - gyro_bias_.z) * GYRO_SCALE;
			}
		} else {
			stationary_time_s_ =
				std::max(0.0, stationary_time_s_ - dt * STATIONARY_DECAY_RATE);
			bias_auto_active_ = false;
		}

		if (a_norm > 1e-6 && std::isfinite(a_norm)) {
			const double axn = ax / a_norm;
			const double ayn = ay / a_norm;
			const double azn = az / a_norm;
			const vec3d g_body =
				rotate_world_vector_into_body(q_, {0.0, -1.0, 0.0});
			const double ex = g_body.y * azn - g_body.z * ayn;
			const double ey = g_body.z * axn - g_body.x * azn;
			const double ez = g_body.x * ayn - g_body.y * axn;
			wx += kp_ * ex;
			wy += kp_ * ey;
			wz += kp_ * ez;
		}

		const quatd qdot = quat_derivative(q_, wx, wy, wz);
		q_.w += qdot.w * dt;
		q_.x += qdot.x * dt;
		q_.y += qdot.y * dt;
		q_.z += qdot.z * dt;
		q_ = quat_normalize(q_);
		latest_omega_ = {wx, wy, wz};

		const quatd qrel = quat_multiply(zq_, q_);
		if (mag_yaw_enabled_)
			try_update_mag_yaw_correction(t, qrel);
	}

	bool compute_mag_heading(double &heading_rad) const
	{
		const mag_sample mag = apply_mount_to_mag(latest_mag_);
		const double mn = std::sqrt(static_cast<double>(mag.mx) * mag.mx +
					    static_cast<double>(mag.my) * mag.my +
					    static_cast<double>(mag.mz) * mag.mz);
		if (!std::isfinite(mn) || mn < 1e-6)
			return false;
		if (std::abs(mag.mx) >= MAG_SAT_ABS || std::abs(mag.my) >= MAG_SAT_ABS ||
		    std::abs(mag.mz) >= MAG_SAT_ABS)
			return false;

		const vec3d g_body = rotate_world_vector_into_body(q_, {0.0, -1.0, 0.0});
		const double gn = std::sqrt(g_body.x * g_body.x + g_body.y * g_body.y +
					    g_body.z * g_body.z);
		if (!std::isfinite(gn) || gn < 1e-6)
			return false;
		const vec3d g = {g_body.x / gn, g_body.y / gn, g_body.z / gn};
		const vec3d m = {mag.mx, mag.my, mag.mz};
		const double dot_gm = m.x * g.x + m.y * g.y + m.z * g.z;
		const vec3d mh = {m.x - dot_gm * g.x, m.y - dot_gm * g.y,
				  m.z - dot_gm * g.z};
		const double hn = std::sqrt(mh.x * mh.x + mh.y * mh.y + mh.z * mh.z);
		if (!std::isfinite(hn) || hn < 1e-6)
			return false;
		heading_rad = std::atan2(mh.x, -mh.z);
		return true;
	}

	void try_update_mag_yaw_correction(uint32_t imu_ts_us, quatd qrel)
	{
		const uint32_t mag_ts = latest_mag_.ts_us;
		if (!mag_ts)
			return;
		const int32_t d_age = static_cast<int32_t>(imu_ts_us - mag_ts);
		const uint32_t age_us = d_age >= 0 ? static_cast<uint32_t>(d_age) : 0;
		if (age_us > MAG_STALE_US)
			return;
		if (mag_ts == last_mag_ts_used_)
			return;

		double heading = 0.0;
		const uint32_t prev_mag_ts = last_mag_ts_used_;
		last_mag_ts_used_ = mag_ts;
		if (!compute_mag_heading(heading))
			return;

		const quatd qrel_corr_now = quat_multiply(mag_yaw_corr_, qrel);
		const double yaw_corr_now = yaw_from_quat_heading(qrel_corr_now);
		if (!mag_ref_valid_) {
			mag_heading_ref_rad_ = heading;
			mag_yaw_ref_rad_ = yaw_corr_now;
			mag_ref_valid_ = true;
			return;
		}

		const uint32_t d_us = prev_mag_ts ? elapsed_us32(mag_ts, prev_mag_ts) : 2500;
		const double dt = static_cast<double>(d_us) / 1000000.0;
		if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.5)
			return;

		const double k = 1.0 - std::exp(-dt / MAG_YAW_TAU_S);
		const double heading_delta = wrap_angle(heading - mag_heading_ref_rad_);
		const double target_yaw = wrap_angle(mag_yaw_ref_rad_ + heading_delta);
		const double err = wrap_angle(target_yaw - yaw_corr_now);
		const quatd q_delta = quat_from_yaw_y(k * err);
		mag_yaw_corr_ = quat_normalize(quat_multiply(q_delta, mag_yaw_corr_));
	}

	double mount_x_deg_ = MOUNT_X_DEG_ONE_STANDARD;
	bool is_calibrated_ = false;
	const uint32_t calibration_min_samples_ = 12;
	const uint32_t calibration_min_duration_us_ = 2000000;
	uint32_t calibration_count_ = 0;
	uint32_t calibration_start_ts_us_ = 0;
	uint32_t calibration_warmup_ts_us_ = 0;
	vec3d gyro_bias_;
	double accel_ref_norm_ = 1.0;
	vec3d sum_g_;
	vec3d sum_g2_;
	vec3d sum_a_;
	double sum_an_ = 0.0;
	double sum_an2_ = 0.0;
	bool debug_log_ = false;
	uint32_t last_imu_ts_us_ = 0;
	double stationary_time_s_ = 0.0;
	bool bias_auto_active_ = false;
	quatd q_;
	quatd zq_;
	double kp_ = 2.0;
	quatd mount_q_;
	bool mag_yaw_enabled_ = false;
	quatd mag_yaw_corr_;
	uint32_t last_mag_ts_used_ = 0;
	bool mag_ref_valid_ = false;
	double mag_heading_ref_rad_ = 0.0;
	double mag_yaw_ref_rad_ = 0.0;
	mag_sample latest_mag_;
	vec3d latest_omega_;
	uint64_t imu_count_ = 0;
	uint64_t mag_count_ = 0;
};
