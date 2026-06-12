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

// Values of device_manager::monitor_out: what the dock does with OBS's audio
// monitoring device.
enum monitor_out_mode {
	MONITOR_OUT_AUTO_GLASSES = 0, // switch to the glasses' USB audio
	MONITOR_OUT_KEEP = 1,         // leave OBS's monitoring device alone
	MONITOR_OUT_DEVICE = 2,       // hold monitoring on a chosen endpoint
};

// Bits of device_manager::dock_collapsed: which dock sections are folded.
enum dock_section_bit : uint32_t {
	DOCK_SECTION_STATUS = 1u << 0,
	DOCK_SECTION_DEVICE = 1u << 1,
	DOCK_SECTION_OUTPUT = 1u << 2,
	DOCK_SECTION_MARKER = 1u << 3,
	DOCK_SECTION_SCREEN = 1u << 4,
	DOCK_SECTION_ADVANCED = 1u << 5,
};
constexpr uint32_t DOCK_COLLAPSED_DEFAULT = DOCK_SECTION_ADVANCED;

struct device_manager {
	std::mutex state_mutex;
	head_tracker tracker;
	pose_snapshot pose;

	std::thread worker;
	std::thread detect_worker;
	std::thread marker_worker;
	std::atomic<bool> stop{false};
	std::atomic<uint32_t> reconnect_epoch{0};
	std::atomic<bool> connected{false};
	std::atomic<bool> connect_enabled{true};
	std::atomic<bool> debug_log{false};
	std::atomic<bool> mag_yaw{false};
	std::atomic<bool> auto_projector{false};
	// OBS audio-monitoring output policy (monitor_out_mode, dock-driven).
	// MONITOR_OUT_DEVICE holds monitoring on the endpoint named below; the
	// selection survives the endpoint being absent (Bluetooth earphones
	// powered off) and is re-applied when it enumerates again.
	std::atomic<int> monitor_out{MONITOR_OUT_AUTO_GLASSES};
	// Keep the mouse cursor off the glasses display (dock-driven LL hook).
	std::atomic<bool> cursor_fence{false};
	// Folded dock sections (dock_section_bit mask, dock-driven).
	std::atomic<uint32_t> dock_collapsed{DOCK_COLLAPSED_DEFAULT};
	// Marker 6DoF: printed-tag size (black square side) in millimeters.
	std::atomic<float> tag_size_mm{80.0f};
	// Set the virtual-screen distance from the measured head-to-tag
	// distance every time the marker origin anchors (i.e. per recenter).
	std::atomic<bool> screen_dist_from_marker{false};
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
	// XREAL Eye camera accessory (One-family control HID).
	// present: -1 unknown/unavailable, 0 absent, 1 attached.
	// uvc: -1 unknown, 0 disabled, 1 both UVC interfaces enabled.
	// request: -1 none, 0 disable UVC, 1 enable UVC. The session consumes
	// it; the glasses re-enumerate USB after the change (brief dropout).
	std::atomic<int> eye_present{-1};
	std::atomic<int> eye_uvc{-1};
	std::atomic<int> eye_request{-1};
	// SBS output rendering of the virtual screen: 0 = auto (active while
	// the glasses display runs a double-wide SBS mode), 1 = on, 2 = off.
	std::atomic<int> sbs_output{0};
	std::atomic<bool> fov_auto{true};
	std::atomic<int> virtual_source_count{0};
	std::atomic<float> prediction_ms{10.0f};
	std::atomic<float> fov_deg{50.0f};
	std::atomic<float> screen_distance_m{DEFAULT_SCREEN_DISTANCE_M};
	std::atomic<float> screen_size_factor{DEFAULT_SCREEN_SIZE_FACTOR};
	std::atomic<float> screen_curve{DEFAULT_SCREEN_CURVE};
	// Interpupillary distance for the SBS per-eye parallax (mm).
	std::atomic<float> ipd_mm{DEFAULT_IPD_MM};
	// Physical half width (m) of the virtual screen, published by the
	// virtual-screen render path every frame (depends on the referenced
	// texture's aspect, FOV and size factor). 0 until a virtual screen
	// renders; the Audio Wall derives exact audio bearings from it.
	std::atomic<float> screen_half_width_m{0.0f};
	// Yaw the render path subtracts from the world so the Display Wall's
	// chosen center display faces forward (degrees, + = the chosen point
	// was right of the wall center). Published with screen_half_width_m;
	// the Audio Wall subtracts it so bearings keep matching the picture.
	std::atomic<float> screen_yaw_offset_deg{0.0f};

	std::mutex settings_mutex;
	std::string ip = "169.254.2.1";
	int port = 52998;
	// Identity of the MONITOR_OUT_DEVICE choice: the WASAPI endpoint id
	// (stable across reconnects) plus the last seen name, kept for display
	// while the endpoint is absent.
	std::string monitor_device_id;
	std::string monitor_device_name;
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
// Marker-6DoF head position (camera position in the tag frame, world axis
// convention) from the marker tracker thread.
void publish_marker_position(device_manager *f, const vec3d &p_tag_world,
			     uint32_t ts_us);
// Current recentered orientation (pose_snapshot.q) for the marker thread's
// planar-pose disambiguation.
quatd device_pose_orientation(device_manager *f);
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
