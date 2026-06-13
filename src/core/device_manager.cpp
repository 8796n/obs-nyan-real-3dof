// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "device_manager.h"

#include <chrono>
#include <cstring>
#include <thread>

#include "device_registry.h"
#include "hid_io.h"
#include "nyan_log.h"
#include "nyan_time.h"
#include "transports.h"

device_manager g_device;

// Actual mode of the EDID-identified glasses display, cached by the dock's
// poll so the video thread never touches the Win32 display APIs. 0 = no
// glasses display present; virtual sources then fall back to the HID-detected
// device's profile resolution.
std::atomic<uint32_t> g_glasses_display_width{0};
std::atomic<uint32_t> g_glasses_display_height{0};

// SBS output: the glasses display runs a double-wide side-by-side mode
// (e.g. 3840x1080, left half = left eye), so the warped view is rendered
// once per eye into each half. 0 = auto: follow the actual glasses display
// mode, treating anything at least three times as wide as tall as SBS
// (16:9 and 16:10 panels doubled give 3.56 / 3.2; ordinary monitors stay
// well below). Manual ON covers half-SBS, which is not detectable from the
// mode; the EDID-gated glasses display detection keeps ultrawide desktop
// monitors out of the auto path.
bool sbs_output_active(uint32_t output_w, uint32_t output_h)
{
	const int mode = g_device.sbs_output.load(std::memory_order_relaxed);
	if (mode == 1)
		return true;
	if (mode == 2)
		return false;
	const uint32_t glasses_w =
		g_glasses_display_width.load(std::memory_order_relaxed);
	return glasses_w != 0 && output_h != 0 && output_w >= output_h * 3;
}

void publish_pose(device_manager *f, bool connected)
{
	std::lock_guard<std::mutex> lk(f->state_mutex);
	f->pose = f->tracker.snapshot();
	f->pose.connected = connected;
}

model_id detected_hid_model(const device_manager *f)
{
	return f->detected_model.load(std::memory_order_relaxed);
}

bool hid_device_ready(const device_manager *f)
{
	return detected_hid_model(f) != MODEL_UNKNOWN;
}

imu_transport detected_transport_for(const device_manager *f)
{
	return profile_for(detected_hid_model(f)).transport;
}

bool publish_sensor_samples(device_manager *f, const imu_sample *imu,
				   const mag_sample *mag)
{
	std::lock_guard<std::mutex> lk(f->state_mutex);
	const model_id m = detected_hid_model(f);
	if (m == MODEL_UNKNOWN) {
		f->pose.connected = false;
		return false;
	}
	const int mount_override =
		f->mount_override_cdeg.load(std::memory_order_relaxed);
	f->tracker.set_mount_deg(mount_override == INT32_MIN
					 ? profile_for(m).mount_x_deg
					 : mount_override / 100.0);
	f->tracker.set_debug(f->debug_log.load(std::memory_order_relaxed));
	f->tracker.set_mag_yaw_enabled(f->mag_yaw.load(std::memory_order_relaxed));
	if (mag)
		f->tracker.on_mag(*mag);
	if (imu)
		f->tracker.on_imu(*imu);
	f->pose = f->tracker.snapshot();
	f->pose.connected = true;
	return true;
}

// Counterpart of publish_sensor_samples for transports that deliver a fused
// orientation instead of raw IMU samples (VITURE).
bool publish_external_pose(device_manager *f, const quatd &device_q,
				  uint32_t ts_us)
{
	std::lock_guard<std::mutex> lk(f->state_mutex);
	const model_id m = detected_hid_model(f);
	if (m == MODEL_UNKNOWN) {
		f->pose.connected = false;
		return false;
	}
	const int mount_override =
		f->mount_override_cdeg.load(std::memory_order_relaxed);
	f->tracker.set_mount_deg(mount_override == INT32_MIN
					 ? profile_for(m).mount_x_deg
					 : mount_override / 100.0);
	f->tracker.set_debug(f->debug_log.load(std::memory_order_relaxed));
	f->tracker.on_external_pose(device_q, ts_us);
	f->pose = f->tracker.snapshot();
	f->pose.connected = true;
	return true;
}

// Drop the gaze-dolly navigation offset (viewer back to the origin). Runs on
// every "square up" action: recenter, recalibrate, model change, reset.
static void clear_viewer_offset(device_manager *f)
{
	f->viewer_offset_x.store(0.0f, std::memory_order_relaxed);
	f->viewer_offset_y.store(0.0f, std::memory_order_relaxed);
	f->viewer_offset_z.store(0.0f, std::memory_order_relaxed);
}

static void reset_tracker_for_model_locked(device_manager *f, model_id m)
{
	clear_viewer_offset(f);
	f->tracker.reset();
	if (m != MODEL_UNKNOWN)
		f->tracker.set_mount_deg(profile_for(m).mount_x_deg);
	f->tracker.set_mag_yaw_enabled(f->mag_yaw.load(std::memory_order_relaxed));
	f->pose = f->tracker.snapshot();
	f->pose.connected = false;
}

// Push a model's display FOV into the global device state so the dock and
// renderer reflect the detected/selected device.
static void apply_auto_fov(device_manager *f, model_id m)
{
	if (m == MODEL_UNKNOWN)
		return;
	f->fov_deg.store(profile_for(m).fov_deg, std::memory_order_relaxed);
}

// Run one HID detection pass and, on a model change, update the detected
// model and (in auto-detect mode with auto FOV) the FOV.
static void run_detection_scan(device_manager *f)
{
	const bool dbg = f->debug_log.load(std::memory_order_relaxed);
	std::string present;
	const model_id m = detect_hid_model(dbg ? &present : nullptr);
	const int prev = f->detected_model.exchange(m, std::memory_order_relaxed);
	if (dbg)
		nyan_log(NYAN_LOG_INFO, "[obs-nyan-real-3dof] HID scan: model=%s present=[%s]",
		     profile_for(m).name.c_str(), present.c_str());
	if (prev == m)
		return;
	if (dbg)
		nyan_log(NYAN_LOG_INFO, "[obs-nyan-real-3dof] HID model changed -> %s%s",
		     profile_for(m).name.c_str(),
		     m == MODEL_UNKNOWN ? " (warp bypassed)" : "");

	f->mount_override_cdeg.store(INT32_MIN, std::memory_order_relaxed);
	{
		std::lock_guard<std::mutex> lk(f->state_mutex);
		reset_tracker_for_model_locked(f, m);
	}

	if (f->fov_auto.load(std::memory_order_relaxed))
		apply_auto_fov(f, m);
}

// HID detection runs on its own thread: a full interface enumeration takes
// tens of milliseconds (SetupDi plus an open and string/caps read per device),
// which is long enough to stall a 1000 Hz IMU read loop past the default HID
// input queue depth and show up as a once-per-second pose hitch. The worker
// and the sessions only read the detected_model atomic this thread maintains.
void detect_worker_fn(device_manager *f)
{
	while (!f->stop.load(std::memory_order_relaxed)) {
		run_detection_scan(f);
		for (int i = 0; i < 20 && !f->stop.load(std::memory_order_relaxed);
		     ++i)
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
}

void maybe_log_sensor_rate(device_manager *f, rate_log_state &st,
				  const char *transport)
{
	if (!f->debug_log.load(std::memory_order_relaxed))
		return;
	const uint64_t now = nyan_now_ns();
	if (st.last_log_ns != 0 && now - st.last_log_ns < 2000000000ULL)
		return;

	pose_snapshot p;
	{
		std::lock_guard<std::mutex> lk(f->state_mutex);
		p = f->pose;
	}
	if (st.last_log_ns == 0) {
		st.last_log_ns = now;
		st.last_imu = p.imu_count;
		st.last_mag = p.mag_count;
		st.last_q = p.q;
		st.have_q = p.calibrated;
		return;
	}
	const double sec = static_cast<double>(now - st.last_log_ns) / 1e9;
	const double imu_hz = (p.imu_count - st.last_imu) / sec;
	const double mag_hz = (p.mag_count - st.last_mag) / sec;
	// |w| is the instantaneous rate (noise floor when still); drift is the
	// integrated pose rotation across the log interval, which is the real
	// perceived drift while the glasses physically sit still.
	const double wn = std::sqrt(p.omega.x * p.omega.x + p.omega.y * p.omega.y +
				    p.omega.z * p.omega.z);
	double drift_deg_s = 0.0;
	if (st.have_q) {
		const quatd dq = quat_normalize(
			quat_multiply(quat_inverse(st.last_q), p.q));
		const double ang =
			2.0 * std::acos(clampd(std::abs(dq.w), 0.0, 1.0));
		drift_deg_s = ang / sec * 180.0 / PI;
	}
	st.last_imu = p.imu_count;
	st.last_mag = p.mag_count;
	st.last_log_ns = now;
	st.last_q = p.q;
	st.have_q = p.calibrated;
	nyan_log(NYAN_LOG_INFO,
	     "[obs-nyan-real-3dof] %s IMU %.0fHz MAG %.0fHz |w|=%.4frad/s "
	     "drift=%.3fdeg/s cal=%d stationary=%d",
	     transport, imu_hz, mag_hz, wn, drift_deg_s, (int)p.calibrated,
	     (int)p.stationary);
}

void worker_fn(device_manager *f)
{
	uint32_t seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	while (!f->stop.load(std::memory_order_relaxed)) {
		if (!f->connect_enabled.load(std::memory_order_relaxed) ||
		    !hid_device_ready(f)) {
			f->connected.store(false, std::memory_order_relaxed);
			publish_pose(f, false);
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
			continue;
		}

		switch (detected_transport_for(f)) {
		case imu_transport::one_bridge_tcp:
			run_one_bridge_tcp_session(f, seen_epoch);
			break;
		case imu_transport::air_hid:
			run_air_hid_session(f, seen_epoch);
			break;
		case imu_transport::rayneo_hid:
			run_rayneo_hid_session(f, seen_epoch);
			break;
		case imu_transport::sensor_api:
			run_sensor_api_session(f, seen_epoch);
			break;
		case imu_transport::rokid_hid:
			run_rokid_hid_session(f, seen_epoch);
			break;
		case imu_transport::viture_hid:
			run_viture_hid_session(f, seen_epoch);
			break;
		case imu_transport::nreal_hid:
			run_nreal_hid_session(f, seen_epoch);
			break;
		case imu_transport::none:
		default:
			f->connected.store(false, std::memory_order_relaxed);
			publish_pose(f, false);
			std::this_thread::sleep_for(std::chrono::milliseconds(250));
			seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
			break;
		}
	}
}

void manager_recenter(device_manager *f)
{
	clear_viewer_offset(f);
	std::lock_guard<std::mutex> lk(f->state_mutex);
	f->tracker.recenter();
	f->pose = f->tracker.snapshot();
	f->pose.connected = f->connected.load(std::memory_order_relaxed);
}

void manager_recalibrate(device_manager *f)
{
	clear_viewer_offset(f);
	std::lock_guard<std::mutex> lk(f->state_mutex);
	f->tracker.restart_calibration();
	f->pose = f->tracker.snapshot();
	f->pose.connected = f->connected.load(std::memory_order_relaxed);
}

void manager_step_screen_distance(device_manager *f, double steps)
{
	const double cur = f->screen_distance_m.load(std::memory_order_relaxed);
	const double next = clampd(cur * std::pow(SCREEN_DISTANCE_STEP_RATIO,
						  steps),
				   MIN_SCREEN_DISTANCE_M,
				   MAX_SCREEN_DISTANCE_M);
	f->screen_distance_m.store(static_cast<float>(next),
				   std::memory_order_relaxed);
}

void manager_dolly_step(device_manager *f, double steps)
{
	pose_snapshot p;
	{
		std::lock_guard<std::mutex> lk(f->state_mutex);
		p = f->pose;
	}
	const double d =
		clampd(f->screen_distance_m.load(std::memory_order_relaxed),
		       MIN_SCREEN_DISTANCE_M, MAX_SCREEN_DISTANCE_M);
	vec3d eye = {f->viewer_offset_x.load(std::memory_order_relaxed),
		     f->viewer_offset_y.load(std::memory_order_relaxed),
		     f->viewer_offset_z.load(std::memory_order_relaxed)};
	// Gaze in the same frame the renderer warps in: the head pose
	// composed with the center-display yaw offset. Without a live pose
	// (no glasses) fall back to walking straight ahead.
	vec3d dir = {0.0, 0.0, -1.0};
	if (p.calibrated && p.connected) {
		quatd q = p.q;
		const double yaw_off =
			f->screen_yaw_offset_deg.load(std::memory_order_relaxed) *
			PI / 180.0;
		if (yaw_off != 0.0)
			q = quat_normalize(
				quat_multiply(quat_from_yaw_y(-yaw_off), q));
		dir = rotate_vector(quat_normalize(q), {0.0, 0.0, -1.0});
	}
	// Remaining gap along the gaze to the surface the warp shader
	// samples. Mirrors the shader's geometry exactly (flat plane at
	// z = -d, or the cylinder of radius d/curve through (0,0,-d) with
	// the same far-root choice), so the asymptotic clamp below can
	// never step through the visible screen.
	const double curve = clampd(
		f->screen_curve.load(std::memory_order_relaxed), 0.0,
		MAX_SCREEN_CURVE);
	double gap = 0.0;
	if (curve <= 0.0001) {
		// Only walk while the gaze points at the plane; looking past
		// its edges still works on purpose (the plane is infinite
		// here and the clamps keep the eye sane).
		if (dir.z > -1e-3)
			return;
		gap = (-d - eye.z) / dir.z;
		if (gap <= 0.0)
			return;
	} else {
		const double radius = d / curve;
		const double center_z = radius - d;
		const double ox = eye.x;
		const double oz = eye.z - center_z;
		const double a = dir.x * dir.x + dir.z * dir.z;
		const double b = 2.0 * (ox * dir.x + oz * dir.z);
		const double c = ox * ox + oz * oz - radius * radius;
		const double disc = b * b - 4.0 * a * c;
		if (a <= 1e-6 || disc < 0.0)
			return; // near-vertical gaze, or misses the cylinder
		gap = (-b + std::sqrt(disc)) / (2.0 * a);
		if (gap <= 1e-4)
			return;
	}
	// Scale the remaining gap on the shared log ratio: an asymptotic
	// approach that never reaches the surface.
	const double next_gap =
		clampd(gap * std::pow(SCREEN_DISTANCE_STEP_RATIO, steps),
		       MIN_DOLLY_GAP_M, MAX_DOLLY_GAP_M);
	const double move = gap - next_gap;
	eye.x = clampd(eye.x + dir.x * move, -MAX_DOLLY_GAP_M, MAX_DOLLY_GAP_M);
	eye.y = clampd(eye.y + dir.y * move, -MAX_DOLLY_GAP_M, MAX_DOLLY_GAP_M);
	eye.z = clampd(eye.z + dir.z * move, -MAX_DOLLY_GAP_M, MAX_DOLLY_GAP_M);
	f->viewer_offset_x.store(static_cast<float>(eye.x),
				 std::memory_order_relaxed);
	f->viewer_offset_y.store(static_cast<float>(eye.y),
				 std::memory_order_relaxed);
	f->viewer_offset_z.store(static_cast<float>(eye.z),
				 std::memory_order_relaxed);
}

void manager_apply_model_settings(device_manager *f)
{
	const model_id m = detected_hid_model(f);
	if (f->fov_auto.load(std::memory_order_relaxed) && m != MODEL_UNKNOWN)
		apply_auto_fov(f, m);
	std::lock_guard<std::mutex> lk(f->state_mutex);
	if (m != MODEL_UNKNOWN)
		f->tracker.set_mount_deg(profile_for(m).mount_x_deg);
	f->tracker.set_mag_yaw_enabled(f->mag_yaw.load(std::memory_order_relaxed));
	f->pose = f->tracker.snapshot();
	f->pose.connected = f->connected.load(std::memory_order_relaxed);
}

void manager_set_mag_yaw(device_manager *f, bool enabled)
{
	f->mag_yaw.store(enabled, std::memory_order_relaxed);
	std::lock_guard<std::mutex> lk(f->state_mutex);
	f->tracker.set_mag_yaw_enabled(enabled);
	f->pose = f->tracker.snapshot();
	f->pose.connected = f->connected.load(std::memory_order_relaxed);
}

void manager_set_connect_enabled(device_manager *f, bool enabled)
{
	const bool prev = f->connect_enabled.exchange(enabled, std::memory_order_relaxed);
	if (prev != enabled)
		f->reconnect_epoch.fetch_add(1, std::memory_order_relaxed);
}

void manager_set_network(device_manager *f, const std::string &ip, int port)
{
	bool reconnect = false;
	{
		std::lock_guard<std::mutex> lk(f->settings_mutex);
		const std::string next_ip = ip.empty() ? "169.254.2.1" : ip;
		const int next_port = port > 0 ? port : 52998;
		reconnect = (f->ip != next_ip) || (f->port != next_port);
		f->ip = next_ip;
		f->port = next_port;
	}
	if (reconnect)
		f->reconnect_epoch.fetch_add(1, std::memory_order_relaxed);
}

void manager_reset_defaults(device_manager *f)
{
	bool reconnect = false;
	{
		std::lock_guard<std::mutex> lk(f->settings_mutex);
		reconnect = (f->ip != "169.254.2.1") || (f->port != 52998);
		f->ip = "169.254.2.1";
		f->port = 52998;
		f->monitor_device_id.clear();
		f->monitor_device_name.clear();
	}
	reconnect = reconnect ||
		    !f->connect_enabled.exchange(true, std::memory_order_relaxed);
	f->fov_auto.store(true, std::memory_order_relaxed);
	f->prediction_ms.store(10.0f, std::memory_order_relaxed);
	f->fov_deg.store(50.0f, std::memory_order_relaxed);
	f->screen_distance_m.store(DEFAULT_SCREEN_DISTANCE_M,
				   std::memory_order_relaxed);
	f->screen_size_factor.store(DEFAULT_SCREEN_SIZE_FACTOR,
				    std::memory_order_relaxed);
	f->screen_curve.store(DEFAULT_SCREEN_CURVE, std::memory_order_relaxed);
	f->ipd_mm.store(DEFAULT_IPD_MM, std::memory_order_relaxed);
	f->convergence_link.store(false, std::memory_order_relaxed);
	f->mag_yaw.store(false, std::memory_order_relaxed);
	f->auto_projector.store(false, std::memory_order_relaxed);
	f->monitor_out.store(MONITOR_OUT_AUTO_GLASSES, std::memory_order_relaxed);
	f->cursor_fence.store(false, std::memory_order_relaxed);
	f->dock_collapsed.store(DOCK_COLLAPSED_DEFAULT, std::memory_order_relaxed);
	f->debug_log.store(false, std::memory_order_relaxed);
	f->sbs_output.store(0, std::memory_order_relaxed);
	f->remote_enabled.store(false, std::memory_order_relaxed);
	f->remote_port.store(DEFAULT_REMOTE_PORT, std::memory_order_relaxed);
	{
		std::lock_guard<std::mutex> lk(f->settings_mutex);
		f->remote_token.clear();
	}

	const model_id m = detected_hid_model(f);
	if (m != MODEL_UNKNOWN)
		f->fov_deg.store(profile_for(m).fov_deg, std::memory_order_relaxed);
	{
		std::lock_guard<std::mutex> lk(f->state_mutex);
		reset_tracker_for_model_locked(f, m);
	}
	if (reconnect)
		f->reconnect_epoch.fetch_add(1, std::memory_order_relaxed);
}

