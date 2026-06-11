// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Global device state: the worker thread that owns all device I/O, the
// tracker it feeds, and the dock-facing setters.
#pragma once

#include <obs-module.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "head_tracker.h"
#include "nyan_types.h"

struct device_manager {
	std::mutex state_mutex;
	head_tracker tracker;
	pose_snapshot pose;

	std::thread worker;
	std::thread detect_worker;
	std::atomic<bool> stop{false};
	std::atomic<uint32_t> reconnect_epoch{0};
	std::atomic<bool> connected{false};
	std::atomic<bool> connect_enabled{true};
	std::atomic<bool> debug_log{false};
	std::atomic<bool> mag_yaw{false};
	std::atomic<bool> auto_projector{false};
	// Switch OBS's audio monitoring device to the glasses' USB audio when a
	// device is detected (one latch per connection, dock-driven).
	std::atomic<bool> auto_monitor{true};
	// Keep the mouse cursor off the glasses display (dock-driven LL hook).
	std::atomic<bool> cursor_fence{false};
	std::atomic<int> detected_model{MODEL_UNKNOWN}; // model_id, set by worker
	// Mount override in centidegrees, reported by the device itself (RayNeo
	// derives it from the device-info board id). INT32_MIN = no override,
	// use the registry profile's mount.
	std::atomic<int> mount_override_cdeg{INT32_MIN};
	// MOVERIO display brightness over the serial command port.
	// brightness current: -1 unknown/unavailable, 0-20 level, 50 auto mode.
	// autobright current: -1 unknown/unavailable, 0 off, 1 on.
	// requests: -1 none; the sensor_api session consumes and applies them.
	std::atomic<int> brightness_current{-1};
	std::atomic<int> brightness_request{-1};
	std::atomic<int> autobright_current{-1};
	std::atomic<int> autobright_request{-1};
	// Display mode over the device command channel (air_hid MI_04 binary
	// protocol / nreal_hid MCU ASCII protocol). Values are the family's
	// protocol mode values from transport_traits.display_modes.
	// current: -1 unknown/unavailable. request: -1 none; the session
	// consumes and applies it, then refreshes current with a GET.
	std::atomic<int> display_mode_current{-1};
	std::atomic<int> display_mode_request{-1};
	// SBS output rendering of the virtual screen: 0 = auto (active while
	// the glasses display runs a double-wide SBS mode), 1 = on, 2 = off.
	std::atomic<int> sbs_output{0};
	std::atomic<bool> fov_auto{true};
	std::atomic<int> virtual_source_count{0};
	std::atomic<float> prediction_ms{10.0f};
	std::atomic<float> fov_deg{50.0f};
	std::atomic<float> screen_distance_m{DEFAULT_SCREEN_DISTANCE_M};
	std::atomic<float> screen_size_factor{1.0f};
	std::atomic<float> screen_curve{DEFAULT_SCREEN_CURVE};
	// Interpupillary distance for the SBS per-eye parallax (mm).
	std::atomic<float> ipd_mm{DEFAULT_IPD_MM};
	// Physical half width (m) of the virtual screen, published by the
	// virtual-screen render path every frame (depends on the referenced
	// texture's aspect, FOV and size factor). 0 until a virtual screen
	// renders; the Audio Wall derives exact audio bearings from it.
	std::atomic<float> screen_half_width_m{0.0f};

	std::mutex settings_mutex;
	std::string ip = "169.254.2.1";
	int port = 52998;
};

struct rate_log_state {
	uint64_t last_log_ns = 0;
	uint64_t last_imu = 0;
	uint64_t last_mag = 0;
	quatd last_q;
	bool have_q = false;
};

extern device_manager g_device;

// Actual mode of the EDID-identified glasses display, cached by the dock's
// poll so the video thread never touches the Win32 display APIs. 0 = no
// glasses display present; virtual sources then fall back to the HID-detected
// device's profile resolution.
extern std::atomic<uint32_t> g_glasses_display_width;
extern std::atomic<uint32_t> g_glasses_display_height;

void publish_pose(device_manager *f, bool connected);
model_id detected_hid_model(const device_manager *f);
bool hid_device_ready(const device_manager *f);
imu_transport detected_transport_for(const device_manager *f);
bool publish_sensor_samples(device_manager *f, const imu_sample *imu,
			    const mag_sample *mag);
bool publish_external_pose(device_manager *f, const quatd &device_q,
			   uint32_t ts_us);
void maybe_log_sensor_rate(device_manager *f, rate_log_state &st,
			   const char *transport);
void worker_fn(device_manager *f);
void detect_worker_fn(device_manager *f);

void manager_apply_settings(device_manager *f, obs_data_t *settings);
void manager_recenter(device_manager *f);
void manager_recalibrate(device_manager *f);
void manager_apply_model_settings(device_manager *f);
void manager_set_mag_yaw(device_manager *f, bool enabled);
void manager_set_connect_enabled(device_manager *f, bool enabled);
void manager_set_network(device_manager *f, const std::string &ip, int port);
void manager_reset_defaults(device_manager *f);
void manager_save_load(obs_data_t *save_data, bool saving, void *private_data);
void recenter_hotkey(void *data, obs_hotkey_id id, obs_hotkey_t *hotkey,
		     bool pressed);
