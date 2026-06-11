// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Shared plain types and module-wide constants. Header-only; no OBS or
// Windows dependencies beyond the standard library.
#pragma once

#include <cstdint>
#include <string>

#ifndef NYAN_REAL_3DOF_VERSION
#define NYAN_REAL_3DOF_VERSION "0.0.0-nocmake"
#endif
#ifndef NYAN_REAL_3DOF_BUILD_TIME
#define NYAN_REAL_3DOF_BUILD_TIME __DATE__ " " __TIME__
#endif

#define BUILD_INFO \
	("obs-nyan-real-3dof " NYAN_REAL_3DOF_VERSION "  (built " NYAN_REAL_3DOF_BUILD_TIME ")")

constexpr float DEFAULT_SCREEN_DISTANCE_M = 4.0f;
// 0.1 m is an experimental extreme for the "walk up to the screen" effect;
// with SBS parallax, distances under ~0.5 m strain the eyes (the convergence
// follows the distance but the glasses' focal plane stays fixed).
constexpr float MIN_SCREEN_DISTANCE_M = 0.1f;
constexpr float MAX_SCREEN_DISTANCE_M = 10.0f;
constexpr float DEFAULT_SCREEN_CURVE = 0.0f;
constexpr float VIRTUAL_TARGET_RETRY_INTERVAL_S = 1.0f;
constexpr float MAX_SCREEN_CURVE = 3.0f;
// Interpupillary distance for SBS per-eye parallax. 63 mm is the adult
// average; the range covers roughly the 1st..99th percentile.
constexpr float DEFAULT_IPD_MM = 63.0f;
constexpr float MIN_IPD_MM = 40.0f;
constexpr float MAX_IPD_MM = 80.0f;

// IMU mount offset as a rotation around X in degrees. Known values: XREAL One
// standard +180, XREAL One Pro -150, XREAL Air 0, RayNeo Air -20. Kept as a
// free angle so devices.json can express new devices without code changes.
constexpr float MOUNT_X_DEG_ONE_STANDARD = 180.0f;
constexpr float MOUNT_X_DEG_ONE_PRO = -150.0f;
constexpr float MOUNT_X_DEG_AIR = 0.0f;
constexpr float MOUNT_X_DEG_RAYNEO = -20.0f;
constexpr float MOUNT_X_DEG_MOVERIO = 0.0f;
// ar-drivers-rs models the Rokid displays as tilted against the IMU:
// 0.07 rad (~4 deg) on the Max, 0.022 rad (~1.3 deg) on the Air. The sign for
// our convention was confirmed on Max hardware.
constexpr float MOUNT_X_DEG_ROKID_MAX = -4.0f;
constexpr float MOUNT_X_DEG_ROKID_AIR = -1.3f;
// XRLinuxDriver's pitch adjustments for the VITURE families.
constexpr float MOUNT_X_DEG_VITURE_ONE = 6.0f;
constexpr float MOUNT_X_DEG_VITURE_PRO = 3.0f;
// ar-drivers-rs models the Nreal Light display as tilted -0.265 rad against
// the IMU; the sign follows the same convention mapping confirmed on Rokid
// hardware (their 0.07 rad = our -4 deg).
constexpr float MOUNT_X_DEG_NREAL_LIGHT = 15.2f;

enum class imu_transport : int {
	none = 0,
	one_bridge_tcp = 1,
	air_hid = 2,
	rayneo_hid = 3,
	sensor_api = 4, // Windows Sensor API (HID sensor collections)
	rokid_hid = 5,
	viture_hid = 6, // fused euler angles over vendor HID
	nreal_hid = 7,  // Nreal Light: raw IMU from the OV580 coprocessor
};

// Device identity. HID detection resolves a present USB device into a 1-based
// index into the device registry; MODEL_UNKNOWN (0) means no supported device.
// The registry is built once in obs_module_load (built-in table plus the
// optional user devices.json) before the worker thread and the dock exist, and
// is immutable afterwards, so every thread reads it without locking.
using model_id = int;
constexpr model_id MODEL_UNKNOWN = 0;

struct model_profile {
	imu_transport transport;
	float mount_x_deg;
	float fov_deg;
	uint32_t display_width;
	uint32_t display_height;
	std::string name;
};

// One registry row: USB identity plus the profile it selects. pid 0 matches
// any PID of the VID; product_contains, when non-empty, must match the HID
// ProductString case-insensitively. First match wins and user entries are
// checked before built-ins, so users can both add devices and override
// built-in profiles from devices.json.
struct device_entry {
	uint16_t vid;
	uint16_t pid; // 0 = any PID
	std::wstring product_contains;
	model_profile profile;
};

// One selectable display mode of a glasses family: the protocol's mode value
// plus the locale key for its dock label. Listed per transport in
// transport_traits; an empty list hides the dock's display-mode row.
struct display_mode_option {
	int value;
	const char *label_key;
};

// Per-transport traits. The dock consults this table instead of hardcoding
// model families, so transport-specific rows follow new devices automatically.
struct transport_traits {
	const char *name_key;       // locale key for the transport display name
	bool uses_network_endpoint; // transport reads the ip/port settings
	bool hid_imu_stream;        // IMU streams over HID input reports
	bool display_brightness;    // brightness set over the serial command port
	// Display modes switchable over the device's command channel; the
	// session consumes display_mode_request and maintains
	// display_mode_current using these protocol values.
	const display_mode_option *display_modes = nullptr;
	size_t display_mode_count = 0;
	// The XREAL Eye camera accessory: the dock shows its attach state and
	// a UVC enable/disable toggle (One-family control HID).
	bool eye_camera = false;
};

struct vec3d {
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

struct quatd {
	double w = 1.0;
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

struct imu_sample {
	float gx = 0.0f;
	float gy = 0.0f;
	float gz = 0.0f;
	float ax = 0.0f;
	float ay = 0.0f;
	float az = 0.0f;
	uint32_t ts_us = 0;
	uint32_t seq = 0;
};

struct mag_sample {
	float mx = 0.0f;
	float my = 0.0f;
	float mz = 0.0f;
	float temp_c = 0.0f;
	uint32_t ts_us = 0;
	uint32_t seq = 0;
};

struct pose_snapshot {
	quatd q;
	vec3d omega;
	uint32_t ts_us = 0;
	bool calibrated = false;
	bool connected = false;
	bool stationary = false;
	uint64_t imu_count = 0;
	uint64_t mag_count = 0;
	// Marker-6DoF head position in the recentered world frame (meters).
	// Holds the last value while the tag is not visible (3DoF fallback).
	vec3d pos;
	bool pos_valid = false;
};

// Shared by the Air and RayNeo HID decoders.
struct decoded_sensor_report {
	imu_sample imu;
	mag_sample mag;
	bool has_imu = false;
	bool has_mag = false;
};
