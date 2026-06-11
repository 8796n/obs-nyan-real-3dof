// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
#include "device_manager.h"

#include <chrono>
#include <cstring>
#include <thread>

#include "device_registry.h"
#include "hid_io.h"
#include "transports/transports.h"

device_manager g_device;

// Actual mode of the EDID-identified glasses display, cached by the dock's
// poll so the video thread never touches the Win32 display APIs. 0 = no
// glasses display present; virtual sources then fall back to the HID-detected
// device's profile resolution.
std::atomic<uint32_t> g_glasses_display_width{0};
std::atomic<uint32_t> g_glasses_display_height{0};

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

static void reset_tracker_for_model_locked(device_manager *f, model_id m)
{
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
		blog(LOG_INFO, "[obs-nyan-real-3dof] HID scan: model=%s present=[%s]",
		     profile_for(m).name.c_str(), present.c_str());
	if (prev == m)
		return;
	if (dbg)
		blog(LOG_INFO, "[obs-nyan-real-3dof] HID model changed -> %s%s",
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
	const uint64_t now = os_gettime_ns();
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
	blog(LOG_INFO,
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

static bool get_bool_setting(obs_data_t *settings, const char *name, bool fallback)
{
	return obs_data_has_user_value(settings, name) ? obs_data_get_bool(settings, name)
						      : fallback;
}

static int get_int_setting(obs_data_t *settings, const char *name, int fallback)
{
	return obs_data_has_user_value(settings, name)
		       ? static_cast<int>(obs_data_get_int(settings, name))
		       : fallback;
}

static double get_double_setting(obs_data_t *settings, const char *name, double fallback)
{
	return obs_data_has_user_value(settings, name) ? obs_data_get_double(settings, name)
						      : fallback;
}

void manager_apply_settings(device_manager *f, obs_data_t *settings)
{
	if (!settings)
		return;

	const char *ip = obs_data_get_string(settings, "ip");
	const int port = get_int_setting(settings, "port", 52998);
	bool reconnect = false;
	{
		std::lock_guard<std::mutex> lk(f->settings_mutex);
		const std::string next_ip = (ip && *ip) ? ip : "169.254.2.1";
		const int next_port = port > 0 ? port : 52998;
		reconnect = (f->ip != next_ip) || (f->port != next_port);
		f->ip = next_ip;
		f->port = next_port;
	}
	const bool next_connect_enabled =
		get_bool_setting(settings, "connect_enabled", true);
	reconnect = reconnect ||
		    (f->connect_enabled.load(std::memory_order_relaxed) !=
		     next_connect_enabled);
	f->connect_enabled.store(next_connect_enabled, std::memory_order_relaxed);
	const bool next_fov_auto = get_bool_setting(settings, "fov_auto", true);
	f->fov_auto.store(next_fov_auto, std::memory_order_relaxed);
	f->prediction_ms.store(
		static_cast<float>(get_double_setting(settings, "prediction_ms", 10.0)),
		std::memory_order_relaxed);
	f->fov_deg.store(static_cast<float>(get_double_setting(settings, "fov_deg", 50.0)),
			 std::memory_order_relaxed);
	const double screen_size_factor =
		get_double_setting(settings, "screen_size_factor", 1.0);
	f->screen_distance_m.store(
		static_cast<float>(get_double_setting(settings, "screen_distance_m",
						      DEFAULT_SCREEN_DISTANCE_M)),
		std::memory_order_relaxed);
	f->screen_size_factor.store(static_cast<float>(screen_size_factor),
				    std::memory_order_relaxed);
	f->screen_curve.store(
		static_cast<float>(get_double_setting(settings, "screen_curve",
						      DEFAULT_SCREEN_CURVE)),
		std::memory_order_relaxed);
	f->ipd_mm.store(static_cast<float>(get_double_setting(settings, "ipd_mm",
							      DEFAULT_IPD_MM)),
			std::memory_order_relaxed);
	f->mag_yaw.store(get_bool_setting(settings, "mag_yaw", false),
			 std::memory_order_relaxed);
	f->auto_projector.store(get_bool_setting(settings, "auto_projector", false),
				std::memory_order_relaxed);
	f->auto_monitor.store(get_bool_setting(settings, "auto_monitor", true),
			      std::memory_order_relaxed);
	f->cursor_fence.store(get_bool_setting(settings, "cursor_fence", false),
			      std::memory_order_relaxed);
	f->debug_log.store(get_bool_setting(settings, "debug_log", false),
			   std::memory_order_relaxed);
	const long long sbs = obs_data_has_user_value(settings, "sbs_output")
				      ? obs_data_get_int(settings, "sbs_output")
				      : 0;
	f->sbs_output.store(sbs >= 0 && sbs <= 2 ? static_cast<int>(sbs) : 0,
			    std::memory_order_relaxed);
	const model_id m = detected_hid_model(f);
	if (next_fov_auto && m != MODEL_UNKNOWN)
		f->fov_deg.store(profile_for(m).fov_deg, std::memory_order_relaxed);
	{
		std::lock_guard<std::mutex> lk(f->state_mutex);
		if (m != MODEL_UNKNOWN)
			f->tracker.set_mount_deg(profile_for(m).mount_x_deg);
		f->tracker.set_mag_yaw_enabled(
			f->mag_yaw.load(std::memory_order_relaxed));
	}
	if (reconnect)
		f->reconnect_epoch.fetch_add(1, std::memory_order_relaxed);
}

void manager_recenter(device_manager *f)
{
	std::lock_guard<std::mutex> lk(f->state_mutex);
	f->tracker.recenter();
	f->pose = f->tracker.snapshot();
	f->pose.connected = f->connected.load(std::memory_order_relaxed);
}

void recenter_hotkey(void *data, obs_hotkey_id, obs_hotkey_t *, bool pressed)
{
	if (!pressed)
		return;
	manager_recenter(&g_device);
}

void manager_recalibrate(device_manager *f)
{
	std::lock_guard<std::mutex> lk(f->state_mutex);
	f->tracker.restart_calibration();
	f->pose = f->tracker.snapshot();
	f->pose.connected = f->connected.load(std::memory_order_relaxed);
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
	}
	reconnect = reconnect ||
		    !f->connect_enabled.exchange(true, std::memory_order_relaxed);
	f->fov_auto.store(true, std::memory_order_relaxed);
	f->prediction_ms.store(10.0f, std::memory_order_relaxed);
	f->fov_deg.store(50.0f, std::memory_order_relaxed);
	f->screen_distance_m.store(DEFAULT_SCREEN_DISTANCE_M,
				   std::memory_order_relaxed);
	f->screen_size_factor.store(1.0f, std::memory_order_relaxed);
	f->screen_curve.store(DEFAULT_SCREEN_CURVE, std::memory_order_relaxed);
	f->ipd_mm.store(DEFAULT_IPD_MM, std::memory_order_relaxed);
	f->mag_yaw.store(false, std::memory_order_relaxed);
	f->auto_projector.store(false, std::memory_order_relaxed);
	f->auto_monitor.store(true, std::memory_order_relaxed);
	f->cursor_fence.store(false, std::memory_order_relaxed);
	f->debug_log.store(false, std::memory_order_relaxed);
	f->sbs_output.store(0, std::memory_order_relaxed);

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

void manager_save_load(obs_data_t *save_data, bool saving, void *)
{
	const char *key = "nyan-real-3dof";
	if (saving) {
		obs_data_t *obj = obs_data_create();
		obs_data_set_bool(obj, "connect_enabled",
				  g_device.connect_enabled.load(std::memory_order_relaxed));
		{
			std::lock_guard<std::mutex> lk(g_device.settings_mutex);
			obs_data_set_string(obj, "ip", g_device.ip.c_str());
			obs_data_set_int(obj, "port", g_device.port);
		}
		obs_data_set_bool(obj, "fov_auto",
				  g_device.fov_auto.load(std::memory_order_relaxed));
		obs_data_set_double(obj, "prediction_ms",
				    g_device.prediction_ms.load(std::memory_order_relaxed));
		obs_data_set_double(obj, "fov_deg",
				    g_device.fov_deg.load(std::memory_order_relaxed));
		obs_data_set_double(
			obj, "screen_distance_m",
			g_device.screen_distance_m.load(std::memory_order_relaxed));
		obs_data_set_double(
			obj, "screen_size_factor",
			g_device.screen_size_factor.load(std::memory_order_relaxed));
		obs_data_set_double(
			obj, "screen_curve",
			g_device.screen_curve.load(std::memory_order_relaxed));
		obs_data_set_double(obj, "ipd_mm",
				    g_device.ipd_mm.load(std::memory_order_relaxed));
		obs_data_set_bool(obj, "mag_yaw",
				  g_device.mag_yaw.load(std::memory_order_relaxed));
		obs_data_set_bool(obj, "auto_projector",
				  g_device.auto_projector.load(std::memory_order_relaxed));
		obs_data_set_bool(obj, "auto_monitor",
				  g_device.auto_monitor.load(std::memory_order_relaxed));
		obs_data_set_bool(obj, "cursor_fence",
				  g_device.cursor_fence.load(std::memory_order_relaxed));
		obs_data_set_bool(obj, "debug_log",
				  g_device.debug_log.load(std::memory_order_relaxed));
		obs_data_set_int(obj, "sbs_output",
				 g_device.sbs_output.load(std::memory_order_relaxed));
		obs_data_set_obj(save_data, key, obj);
		obs_data_release(obj);
		return;
	}

	obs_data_t *obj = obs_data_get_obj(save_data, key);
	if (obj) {
		manager_apply_settings(&g_device, obj);
		obs_data_release(obj);
	}
}
