// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
/*
 * obs-nyan-real-3dof: HID-gated AR-glasses IMU -> global 3DoF pose + source warp.
 *
 * The render thread never touches device I/O. A module-level worker
 * (device_manager.cpp) identifies the glasses over HID (hid_io.cpp +
 * device_registry.cpp), reads the current IMU stream (transports/), updates a
 * small IMU tracker (head_tracker.h), and publishes the latest pose snapshot.
 * The dock (dock.cpp) owns device and screen settings; the virtual-screen
 * input source (virtual_source.cpp) only samples that global state and runs a
 * backward warp shader over a texture. This file is just the module entry
 * point.
 */
#include <obs-frontend-api.h>
#include <obs-module.h>

#include <cstdarg>
#include <thread>

#include "audio-wall-source.h"
#include "device_manager.h"
#include "device_registry.h"
#include "device_settings_obs.h"
#include "display-wall-source.h"
#include "dock_host_obs.h"
#include "nyan_host.h"
#include "nyan_log.h"
#include "nyan_paths.h"
#include "nyan_types.h"
#include "remote_control.h"
#include "spatial_audio_filter.h"
#include "virtual_source.h"

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("obs-nyan-real-3dof", "en-US")

MODULE_EXPORT const char *obs_module_description(void)
{
	return "nyan Real 3DoF head-tracked virtual screen (HID-detected XREAL/RayNeo devices)";
}

static obs_hotkey_id g_recenter_hotkey_id = OBS_INVALID_HOTKEY_ID;

// Route core (OBS-independent) logging into the OBS log. Core uses its own
// NYAN_LOG_* values, so map them to libobs LOG_* here rather than assuming a
// shared numbering.
static void nyan_obs_log_sink(int level, const char *fmt, va_list args)
{
	int obs_level;
	switch (level) {
	case NYAN_LOG_ERROR:
		obs_level = LOG_ERROR;
		break;
	case NYAN_LOG_WARNING:
		obs_level = LOG_WARNING;
		break;
	case NYAN_LOG_DEBUG:
		obs_level = LOG_DEBUG;
		break;
	default:
		obs_level = LOG_INFO;
		break;
	}
	blogva(obs_level, fmt, args);
}

// Resolve core file lookups to OBS module paths: config dir for writable files
// (devices.json), the bundled data dir for read-only assets (remote.html).
static std::string nyan_obs_config_path(const char *filename)
{
	char *p = obs_module_config_path(filename);
	std::string s = p ? p : "";
	bfree(p);
	return s;
}
static std::string nyan_obs_asset_path(const char *filename)
{
	char *p = obs_module_file(filename);
	std::string s = p ? p : "";
	bfree(p);
	return s;
}

// Forward core's audio-output enumeration to OBS's monitoring-device enum.
static void nyan_obs_enum_audio(nyan_audio_output_cb cb, void *data)
{
	obs_enum_audio_monitoring_devices(cb, data);
}

// Bridge core's audio-monitor hooks to OBS's monitoring-device control.
static void nyan_obs_audio_monitor_get(std::string &name, std::string &id)
{
	const char *n = nullptr;
	const char *i = nullptr;
	obs_get_audio_monitoring_device(&n, &i);
	name = n ? n : "";
	id = i ? i : "";
}

static bool nyan_obs_audio_monitor_set(const std::string &name,
				       const std::string &id)
{
	return obs_set_audio_monitoring_device(name.c_str(), id.c_str());
}

static void nyan_obs_audio_monitor_reset()
{
	obs_reset_audio_monitoring();
}

bool obs_module_load(void)
{
	nyan_set_log_sink(nyan_obs_log_sink);
	nyan_set_path_resolvers(nyan_obs_config_path, nyan_obs_asset_path);
	nyan_set_text_provider(obs_module_text);
	nyan_set_locale_provider(obs_get_locale);
	nyan_set_audio_output_enumerator(nyan_obs_enum_audio);
	nyan_set_audio_monitor_getter(nyan_obs_audio_monitor_get);
	nyan_set_audio_monitor_setter(nyan_obs_audio_monitor_set);
	nyan_set_audio_monitor_resetter(nyan_obs_audio_monitor_reset);
	register_dock_host_obs();
	init_device_registry();

	register_nyan_real_virtual_source();
	register_nyan_real_display_wall_source();
	register_nyan_real_ws_audio_source();
	register_nyan_real_spatial_audio_filter();

	g_recenter_hotkey_id = obs_hotkey_register_frontend(
		"nyan_real_3dof.recenter", obs_module_text("hotkey.recenter"),
		recenter_hotkey, nullptr);

	obs_frontend_add_save_callback(manager_save_load, nullptr);
	init_dock();
	g_device.detect_worker = std::thread(detect_worker_fn, &g_device);
	g_device.worker = std::thread(worker_fn, &g_device);

	blog(LOG_INFO, "[obs-nyan-real-3dof] loaded: %s (libobs %d.%d.%d)", BUILD_INFO,
	     LIBOBS_API_MAJOR_VER, LIBOBS_API_MINOR_VER, LIBOBS_API_PATCH_VER);
	return true;
}

void obs_module_unload(void)
{
	shutdown_dock();
	remote_control_shutdown();
	if (g_recenter_hotkey_id != OBS_INVALID_HOTKEY_ID)
		obs_hotkey_unregister(g_recenter_hotkey_id);
	obs_frontend_remove_save_callback(manager_save_load, nullptr);
	g_device.stop.store(true, std::memory_order_relaxed);
	g_device.reconnect_epoch.fetch_add(1, std::memory_order_relaxed);
	if (g_device.worker.joinable())
		g_device.worker.join();
	if (g_device.detect_worker.joinable())
		g_device.detect_worker.join();
	blog(LOG_INFO, "[obs-nyan-real-3dof] unloaded");
}
