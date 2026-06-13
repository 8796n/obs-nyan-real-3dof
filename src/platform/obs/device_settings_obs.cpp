// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
#include "device_settings_obs.h"

#include <mutex>
#include <string>

#include "device_registry.h"
#include "remote_control.h"

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
	const double screen_size_factor = get_double_setting(
		settings, "screen_size_factor", DEFAULT_SCREEN_SIZE_FACTOR);
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
	f->convergence_link.store(
		get_bool_setting(settings, "convergence_link", false),
		std::memory_order_relaxed);
	f->mag_yaw.store(get_bool_setting(settings, "mag_yaw", false),
			 std::memory_order_relaxed);
	f->auto_projector.store(get_bool_setting(settings, "auto_projector", false),
				std::memory_order_relaxed);
	// Monitoring output: settings older than the selector only carry the
	// auto_monitor bool (true = auto-switch to glasses, false = leave be).
	int monitor_out = obs_data_has_user_value(settings, "monitor_out")
				  ? get_int_setting(settings, "monitor_out",
						    MONITOR_OUT_AUTO_GLASSES)
				  : (get_bool_setting(settings, "auto_monitor", true)
					     ? MONITOR_OUT_AUTO_GLASSES
					     : MONITOR_OUT_KEEP);
	if (monitor_out < MONITOR_OUT_AUTO_GLASSES ||
	    monitor_out > MONITOR_OUT_DEVICE)
		monitor_out = MONITOR_OUT_AUTO_GLASSES;
	{
		std::lock_guard<std::mutex> lk(f->settings_mutex);
		if (obs_data_has_user_value(settings, "monitor_device_id")) {
			f->monitor_device_id =
				obs_data_get_string(settings, "monitor_device_id");
			f->monitor_device_name = obs_data_get_string(
				settings, "monitor_device_name");
		}
		// A device choice without an id cannot be applied; fall back.
		if (monitor_out == MONITOR_OUT_DEVICE &&
		    f->monitor_device_id.empty())
			monitor_out = MONITOR_OUT_AUTO_GLASSES;
	}
	f->monitor_out.store(monitor_out, std::memory_order_relaxed);
	f->cursor_fence.store(get_bool_setting(settings, "cursor_fence", false),
			      std::memory_order_relaxed);
	f->dock_collapsed.store(
		static_cast<uint32_t>(get_int_setting(
			settings, "dock_collapsed",
			static_cast<int>(DOCK_COLLAPSED_DEFAULT))),
		std::memory_order_relaxed);
	f->debug_log.store(get_bool_setting(settings, "debug_log", false),
			   std::memory_order_relaxed);
	const long long sbs = obs_data_has_user_value(settings, "sbs_output")
				      ? obs_data_get_int(settings, "sbs_output")
				      : 0;
	f->sbs_output.store(sbs >= 0 && sbs <= 2 ? static_cast<int>(sbs) : 0,
			    std::memory_order_relaxed);
	f->remote_enabled.store(get_bool_setting(settings, "remote_enabled",
						 false),
				std::memory_order_relaxed);
	const int remote_port =
		get_int_setting(settings, "remote_port", DEFAULT_REMOTE_PORT);
	f->remote_port.store(remote_port >= 1024 && remote_port <= 65535
				     ? remote_port
				     : DEFAULT_REMOTE_PORT,
			     std::memory_order_relaxed);
	{
		std::lock_guard<std::mutex> lk(f->settings_mutex);
		f->remote_token =
			obs_data_get_string(settings, "remote_token");
	}
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
	// Bring the phone-remote server up/down promptly on settings loads;
	// the dock's poll keeps reconciling afterwards.
	remote_control_sync();
}

void recenter_hotkey(void *data, obs_hotkey_id, obs_hotkey_t *, bool pressed)
{
	if (!pressed)
		return;
	manager_recenter(&g_device);
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
		obs_data_set_bool(obj, "convergence_link",
				  g_device.convergence_link.load(
					  std::memory_order_relaxed));
		obs_data_set_bool(obj, "mag_yaw",
				  g_device.mag_yaw.load(std::memory_order_relaxed));
		obs_data_set_bool(obj, "auto_projector",
				  g_device.auto_projector.load(std::memory_order_relaxed));
		const int monitor_out =
			g_device.monitor_out.load(std::memory_order_relaxed);
		obs_data_set_int(obj, "monitor_out", monitor_out);
		{
			std::lock_guard<std::mutex> lk(g_device.settings_mutex);
			obs_data_set_string(obj, "monitor_device_id",
					    g_device.monitor_device_id.c_str());
			obs_data_set_string(obj, "monitor_device_name",
					    g_device.monitor_device_name.c_str());
		}
		// Downgrade compatibility: older builds only read this bool.
		obs_data_set_bool(obj, "auto_monitor",
				  monitor_out == MONITOR_OUT_AUTO_GLASSES);
		obs_data_set_bool(obj, "cursor_fence",
				  g_device.cursor_fence.load(std::memory_order_relaxed));
		obs_data_set_int(obj, "dock_collapsed",
				 g_device.dock_collapsed.load(
					 std::memory_order_relaxed));
		obs_data_set_bool(obj, "debug_log",
				  g_device.debug_log.load(std::memory_order_relaxed));
		obs_data_set_int(obj, "sbs_output",
				 g_device.sbs_output.load(std::memory_order_relaxed));
		obs_data_set_bool(obj, "remote_enabled",
				  g_device.remote_enabled.load(
					  std::memory_order_relaxed));
		obs_data_set_int(obj, "remote_port",
				 g_device.remote_port.load(
					 std::memory_order_relaxed));
		{
			std::lock_guard<std::mutex> lk(g_device.settings_mutex);
			obs_data_set_string(obj, "remote_token",
					    g_device.remote_token.c_str());
		}
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
