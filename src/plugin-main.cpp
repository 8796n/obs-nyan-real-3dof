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

#include <thread>

#include "audio-wall-source.h"
#include "device_manager.h"
#include "device_registry.h"
#include "display-wall-source.h"
#include "dock.h"
#include "nyan_types.h"
#include "spatial_audio_filter.h"
#include "virtual_source.h"

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("obs-nyan-real-3dof", "en-US")

MODULE_EXPORT const char *obs_module_description(void)
{
	return "nyan Real 3DoF head-tracked virtual screen (HID-detected XREAL/RayNeo devices)";
}

static obs_hotkey_id g_recenter_hotkey_id = OBS_INVALID_HOTKEY_ID;

bool obs_module_load(void)
{
	init_device_registry();

	register_nyan_real_virtual_source();
	register_nyan_real_display_wall_source();
	register_nyan_real_audio_wall_source();
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
