// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
/*
 * obs-nyan-real-3dof: HID-gated AR-glasses IMU -> global 3DoF pose + source warp.
 *
 * The render thread never touches device I/O. A module-level worker identifies
 * the glasses over HID, reads the current IMU stream, updates a small IMU
 * tracker, and publishes the latest pose snapshot. The dock owns device and
 * screen settings; the virtual-screen input source only samples that global
 * state and runs a backward warp shader over a texture.
 */
#include "display-wall-source.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <hidsdi.h>
#include <setupapi.h>
#include <sensorsapi.h>
#include <sensors.h>
#include <portabledevicetypes.h>

#include <obs-frontend-api.h>
#include <obs-module.h>
#include <graphics/graphics.h>
#include <graphics/vec2.h>
#include <graphics/vec4.h>
#include <util/platform.h>

#include <initguid.h>
// Not in a header we already pull in; value from ntddser.h.
DEFINE_GUID(GUID_DEVINTERFACE_COMPORT, 0x86e0d1e0, 0x8089, 0x11d0, 0x9c, 0xe4,
	    0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73);

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cwctype>
#include <cwchar>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef NYAN_REAL_3DOF_WITH_QT_DOCK
#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QDockWidget>
#include <QFormLayout>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPointer>
#include <QPushButton>
#include <QScreen>
#include <QSignalBlocker>
#include <QSlider>
#include <QScrollArea>
#include <QSize>
#include <QSizePolicy>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QWheelEvent>
#endif

#ifndef NYAN_REAL_3DOF_VERSION
#define NYAN_REAL_3DOF_VERSION "0.0.0-nocmake"
#endif
#ifndef NYAN_REAL_3DOF_BUILD_TIME
#define NYAN_REAL_3DOF_BUILD_TIME __DATE__ " " __TIME__
#endif

#define BUILD_INFO \
	("obs-nyan-real-3dof " NYAN_REAL_3DOF_VERSION "  (built " NYAN_REAL_3DOF_BUILD_TIME ")")

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("obs-nyan-real-3dof", "en-US")

MODULE_EXPORT const char *obs_module_description(void)
{
	return "nyan Real 3DoF head-tracked virtual screen (HID-detected XREAL/RayNeo devices)";
}

namespace {

constexpr uint8_t RAW_HDR[] = {0x28, 0x36, 0x00, 0x00, 0x00, 0x80};
constexpr size_t RAW_HDR_LEN = sizeof(RAW_HDR);
constexpr size_t RAW_RECORD_LEN = 134;
constexpr uint32_t KIND_IMU = 11;
constexpr uint32_t KIND_MAG = 4;
constexpr uint16_t XREAL_VID = 0x3318;
constexpr uint16_t ASUS_VID = 0x0B05;
constexpr uint16_t RAYNEO_VID = 0x1BBB;
constexpr uint16_t EPSON_VID = 0x04B8;
constexpr uint16_t ROKID_VID = 0x04D2;
// Rokid packet types on the vendor HID interface (64-byte fixed packets).
constexpr uint8_t ROKID_PKT_SENSOR = 4;
constexpr uint8_t ROKID_PKT_COMBINED = 17;
constexpr uint16_t VITURE_VID = 0x35CA;
// VITURE 64-byte packets: FF FC = IMU data, FF FE = MCU command; command
// 0x15 with payload 1/0 starts/stops the fused-euler stream.
constexpr uint8_t VITURE_HDR_IMU = 0xFC;
constexpr uint8_t VITURE_HDR_CMD = 0xFE;
constexpr uint16_t VITURE_CMD_IMU = 0x15;
constexpr uint8_t AIR_MSG_START_IMU_DATA = 0x19;
constexpr uint8_t AIR_MSG_GET_STATIC_ID = 0x1A;
// RayNeo HID protocol, mirroring xrealonenet/3dof/js/rayneo_driver.js.
// Commands go out as a 64-byte report (id 0): [0x66, cmd, arg, 0...]. Input
// reports carry a byte stream of [0x99, kind, len, ...] frames.
constexpr uint8_t RAYNEO_CMD_HEADER = 0x66;
constexpr uint8_t RAYNEO_CMD_ACQUIRE_DEVICE_INFO = 0x00;
constexpr uint8_t RAYNEO_CMD_OPEN_IMU = 0x01;
constexpr uint8_t RAYNEO_CMD_CLOSE_IMU = 0x02;
constexpr uint8_t RAYNEO_PKT_MAGIC = 0x99;
constexpr uint8_t RAYNEO_PKT_SENSOR = 0x65;
constexpr uint8_t RAYNEO_PKT_RESPONSE = 0xC8;

constexpr double PI = 3.14159265358979323846;
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
constexpr float DEFAULT_SCREEN_DISTANCE_M = 4.0f;
constexpr float DEFAULT_SCREEN_CURVE = 0.0f;
constexpr float VIRTUAL_TARGET_RETRY_INTERVAL_S = 1.0f;
constexpr float MAX_SCREEN_CURVE = 3.0f;

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

enum class imu_transport : int {
	none = 0,
	one_bridge_tcp = 1,
	air_hid = 2,
	rayneo_hid = 3,
	sensor_api = 4, // Windows Sensor API (HID sensor collections)
	rokid_hid = 5,
	viture_hid = 6, // fused euler angles over vendor HID
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

static std::vector<device_entry> g_device_registry;

static const model_profile &profile_for(model_id m)
{
	static const model_profile unknown_profile = {imu_transport::none,
						      MOUNT_X_DEG_ONE_STANDARD,
						      50.0f, 1920, 1080, "Unknown"};
	if (m <= MODEL_UNKNOWN ||
	    static_cast<size_t>(m) > g_device_registry.size())
		return unknown_profile;
	return g_device_registry[static_cast<size_t>(m) - 1].profile;
}

// Per-transport traits. The dock consults this table instead of hardcoding
// model families, so transport-specific rows follow new devices automatically.
struct transport_traits {
	const char *name_key;       // locale key for the transport display name
	bool uses_network_endpoint; // transport reads the ip/port settings
	bool hid_imu_stream;        // IMU streams over HID input reports
	bool display_brightness;    // brightness set over the serial command port
};

static transport_traits traits_for(imu_transport t)
{
	switch (t) {
	case imu_transport::one_bridge_tcp:
		return {"transport.one_bridge_tcp", true, false, false};
	case imu_transport::air_hid:
		return {"transport.air_hid", false, true, false};
	case imu_transport::rayneo_hid:
		return {"transport.rayneo_hid", false, true, false};
	case imu_transport::sensor_api:
		// IMU streams through the Sensor API rather than raw HID
		// reports. MOVERIO has no hardware brightness control; the
		// dock exposes it through the device's serial command port.
		return {"transport.sensor_api", false, false, true};
	case imu_transport::rokid_hid:
		return {"transport.rokid_hid", false, true, false};
	case imu_transport::viture_hid:
		return {"transport.viture_hid", false, true, false};
	case imu_transport::none:
	default:
		return {"transport.none", false, false, false};
	}
}

// Built-in device table. USB IDs from xrealonenet/3dof/js/app.js and
// rayneo_driver.js. XREAL Ultra is intentionally omitted (left as unknown).
// PID 0x0426 appears as both Ultra and Air in historical tables; the trailing
// XREAL wildcard row classifies any other XREAL PID as Air-family only when
// the product string contains "Air", which covers 0x0426 and future
// Air-protocol PIDs. The RayNeo wildcard row mirrors the reference driver's
// /rayneo/i product-name fallback. RayNeo FOV is not stated by the reference
// driver; 46 deg matches the published Air-series specs.
static void append_builtin_devices(std::vector<device_entry> &out)
{
	const model_profile one = {imu_transport::one_bridge_tcp,
				   MOUNT_X_DEG_ONE_STANDARD, 50.0f, 1920, 1080,
				   "XREAL One"};
	const model_profile one_pro = {imu_transport::one_bridge_tcp,
				       MOUNT_X_DEG_ONE_PRO, 57.0f, 1920, 1080,
				       "XREAL One Pro"};
	const model_profile one_s = {imu_transport::one_bridge_tcp,
				     MOUNT_X_DEG_ONE_STANDARD, 52.0f, 1920, 1080,
				     "XREAL 1S"};
	// ROG XREAL R1 is a One Pro derivative: One Pro mount offset and 57 deg
	// diagonal FOV. The reference driver's useProOffset: false is
	// intentionally not followed here.
	const model_profile rog_r1 = {imu_transport::one_bridge_tcp,
				      MOUNT_X_DEG_ONE_PRO, 57.0f, 1920, 1080,
				      "ROG XREAL R1"};
	const model_profile air = {imu_transport::air_hid, MOUNT_X_DEG_AIR,
				   46.0f, 1920, 1080, "XREAL Air / Air 2"};
	const model_profile rayneo = {imu_transport::rayneo_hid,
				      MOUNT_X_DEG_RAYNEO, 46.0f, 1920, 1080,
				      "RayNeo Air"};
	// EPSON MOVERIO BT-40 exposes its IMU as standard HID sensor
	// collections, read through the Windows Sensor API. 34 deg diagonal
	// FOV per the published spec.
	const model_profile bt40 = {imu_transport::sensor_api,
				    MOUNT_X_DEG_MOVERIO, 34.0f, 1920, 1080,
				    "EPSON MOVERIO BT-40"};
	// Rokid Max / Air stream fixed 64-byte packets on a vendor HID
	// interface without any start command. Both share PID 0x162F and are
	// told apart by the product string containing "Max" (mirrors the
	// reference driver). The published Max FOV of 50 deg diagonal applies
	// to the native 1920x1200 panel; at the usual 1920x1080 mode the
	// letterboxed image spans atan(diag1080/diag1200 * tan(25 deg)) * 2 =
	// 48.8 deg. The Air has a native 1080p panel, so its published 43 deg
	// applies directly.
	const model_profile rokid_max = {imu_transport::rokid_hid,
					 MOUNT_X_DEG_ROKID_MAX, 48.8f, 1920,
					 1080, "Rokid Max"};
	const model_profile rokid_air = {imu_transport::rokid_hid,
					 MOUNT_X_DEG_ROKID_AIR, 43.0f, 1920,
					 1080, "Rokid Air"};
	// VITURE glasses fuse orientation on-board and stream euler angles
	// over a vendor HID interface after an enable command. FOV and pitch
	// adjustments follow XRLinuxDriver's metadata (One 40 deg / +6 deg,
	// Pro 43 deg / +3 deg).
	const model_profile viture_one = {imu_transport::viture_hid,
					  MOUNT_X_DEG_VITURE_ONE, 40.0f, 1920,
					  1080, "VITURE One"};
	const model_profile viture_one_lite = {imu_transport::viture_hid,
					       MOUNT_X_DEG_VITURE_ONE, 40.0f,
					       1920, 1080, "VITURE One Lite"};
	const model_profile viture_pro = {imu_transport::viture_hid,
					  MOUNT_X_DEG_VITURE_PRO, 43.0f, 1920,
					  1080, "VITURE Pro"};

	out.push_back({XREAL_VID, 0x0435, L"", one_pro});
	out.push_back({XREAL_VID, 0x0436, L"", one_pro});
	out.push_back({XREAL_VID, 0x0437, L"", one});
	out.push_back({XREAL_VID, 0x0438, L"", one});
	out.push_back({XREAL_VID, 0x043D, L"", one_s});
	out.push_back({XREAL_VID, 0x043E, L"", one_s});
	out.push_back({ASUS_VID, 0x1D9C, L"", rog_r1});
	out.push_back({ASUS_VID, 0x1D9D, L"", rog_r1});
	out.push_back({XREAL_VID, 0x0424, L"", air});
	out.push_back({XREAL_VID, 0x0432, L"", air});
	out.push_back({XREAL_VID, 0x0428, L"", air});
	out.push_back({XREAL_VID, 0x0000, L"Air", air});
	out.push_back({RAYNEO_VID, 0xAF50, L"", rayneo});
	out.push_back({RAYNEO_VID, 0x0000, L"RayNeo", rayneo});
	out.push_back({EPSON_VID, 0x0D12, L"", bt40});
	out.push_back({ROKID_VID, 0x162F, L"Max", rokid_max});
	out.push_back({ROKID_VID, 0x162F, L"", rokid_air});
	out.push_back({ROKID_VID, 0x0000, L"Rokid", rokid_max});
	out.push_back({VITURE_VID, 0x1011, L"", viture_one});
	out.push_back({VITURE_VID, 0x1013, L"", viture_one});
	out.push_back({VITURE_VID, 0x1017, L"", viture_one});
	out.push_back({VITURE_VID, 0x1015, L"", viture_one_lite});
	out.push_back({VITURE_VID, 0x101B, L"", viture_one_lite});
	out.push_back({VITURE_VID, 0x1019, L"", viture_pro});
	out.push_back({VITURE_VID, 0x101D, L"", viture_pro});
	out.push_back({VITURE_VID, 0x0000, L"VITURE", viture_one});
}

struct hid_interface_info {
	std::wstring path;
	std::wstring product;
	uint16_t vid = 0;
	uint16_t pid = 0;
	USAGE usage_page = 0;
	USAGE usage = 0;
	USHORT input_report_len = 0;
	USHORT output_report_len = 0;
	model_id model = MODEL_UNKNOWN;
};

static bool wcontains_ascii_ci(const std::wstring &s, const wchar_t *needle)
{
	if (!needle || !*needle)
		return true;
	const size_t n = std::wcslen(needle);
	if (s.size() < n)
		return false;
	for (size_t i = 0; i + n <= s.size(); ++i) {
		bool ok = true;
		for (size_t j = 0; j < n; ++j) {
			if (std::towupper(s[i + j]) != std::towupper(needle[j])) {
				ok = false;
				break;
			}
		}
		if (ok)
			return true;
	}
	return false;
}

static bool is_consumer_control_hid(const hid_interface_info &info)
{
	return info.usage_page == 0x0C;
}

static model_id model_for_hid_interface(const hid_interface_info &info)
{
	for (size_t i = 0; i < g_device_registry.size(); ++i) {
		const device_entry &e = g_device_registry[i];
		if (info.vid != e.vid)
			continue;
		if (e.pid != 0 && info.pid != e.pid)
			continue;
		// HID IMU streaming runs over non-consumer-control HID
		// interfaces; don't let a Consumer Control collection identify
		// the device either.
		if (traits_for(e.profile.transport).hid_imu_stream &&
		    is_consumer_control_hid(info))
			continue;
		if (!e.product_contains.empty() &&
		    !wcontains_ascii_ci(info.product, e.product_contains.c_str()))
			continue;
		return static_cast<model_id>(i + 1);
	}
	return MODEL_UNKNOWN;
}

static bool read_hid_caps(HANDLE h, hid_interface_info &info)
{
	PHIDP_PREPARSED_DATA preparsed = nullptr;
	if (!HidD_GetPreparsedData(h, &preparsed))
		return false;
	HIDP_CAPS caps = {};
	const NTSTATUS status = HidP_GetCaps(preparsed, &caps);
	HidD_FreePreparsedData(preparsed);
	if (status != HIDP_STATUS_SUCCESS)
		return false;
	info.usage_page = caps.UsagePage;
	info.usage = caps.Usage;
	info.input_report_len = caps.InputReportByteLength;
	info.output_report_len = caps.OutputReportByteLength;
	return true;
}

static std::vector<hid_interface_info> enumerate_hid_interfaces()
{
	std::vector<hid_interface_info> out;
	GUID hid_guid;
	HidD_GetHidGuid(&hid_guid);
	HDEVINFO dev_info = SetupDiGetClassDevsW(
		&hid_guid, nullptr, nullptr,
		DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
	if (dev_info == INVALID_HANDLE_VALUE)
		return out;

	SP_DEVICE_INTERFACE_DATA if_data = {};
	if_data.cbSize = sizeof(if_data);
	for (DWORD i = 0; SetupDiEnumDeviceInterfaces(dev_info, nullptr, &hid_guid, i, &if_data);
	     ++i) {
		DWORD needed = 0;
		SetupDiGetDeviceInterfaceDetailW(dev_info, &if_data, nullptr, 0,
						 &needed, nullptr);
		if (needed == 0)
			continue;
		std::vector<uint8_t> buf(needed);
		auto *detail =
			reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W *>(buf.data());
		detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
		if (!SetupDiGetDeviceInterfaceDetailW(dev_info, &if_data, detail,
						      needed, nullptr, nullptr))
			continue;

		hid_interface_info info;
		info.path = detail->DevicePath;
		HANDLE h = CreateFileW(detail->DevicePath, 0,
				       FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
				       OPEN_EXISTING, 0, nullptr);
		if (h == INVALID_HANDLE_VALUE)
			continue;
		HIDD_ATTRIBUTES attrs = {};
		attrs.Size = sizeof(attrs);
		if (HidD_GetAttributes(h, &attrs)) {
			info.vid = attrs.VendorID;
			info.pid = attrs.ProductID;
			wchar_t product[128] = {};
			if (HidD_GetProductString(h, product, sizeof(product)))
				info.product = product;
			read_hid_caps(h, info);
			info.model = model_for_hid_interface(info);
			out.push_back(info);
		}
		CloseHandle(h);
	}

	SetupDiDestroyDeviceInfoList(dev_info);
	return out;
}

// Enumerate present HID interfaces and return the first known model. When
// out_present is non-null, it is filled with every enumerated VID:PID plus usage
// page for debug logging.
// Sensor-API devices (e.g. MOVERIO BT-40) expose their IMU as HID sensor
// collections that the Windows Sensor class driver claims exclusively, so they
// never appear in the raw-HID enumeration above. Detect them instead by asking
// the Sensor API whether a gyrometer whose device path carries a registered
// sensor_api VID/PID is present. out_present (debug) gets the matched id.
static model_id detect_sensor_api_model(std::string *out_present = nullptr)
{
	bool any = false;
	for (const auto &e : g_device_registry) {
		if (e.profile.transport == imu_transport::sensor_api) {
			any = true;
			break;
		}
	}
	if (!any)
		return MODEL_UNKNOWN;

	const HRESULT co_hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
	const bool co_init = SUCCEEDED(co_hr);
	ISensorManager *mgr = nullptr;
	model_id found = MODEL_UNKNOWN;
	if (SUCCEEDED(CoCreateInstance(CLSID_SensorManager, nullptr,
				       CLSCTX_INPROC_SERVER,
				       IID_PPV_ARGS(&mgr))) &&
	    mgr) {
		ISensorCollection *col = nullptr;
		if (SUCCEEDED(mgr->GetSensorsByType(SENSOR_TYPE_GYROMETER_3D,
						    &col)) &&
		    col) {
			ULONG count = 0;
			col->GetCount(&count);
			for (ULONG c = 0; c < count && found == MODEL_UNKNOWN;
			     ++c) {
				ISensor *s = nullptr;
				if (FAILED(col->GetAt(c, &s)) || !s)
					continue;
				PROPVARIANT pv;
				PropVariantInit(&pv);
				std::wstring path;
				if (SUCCEEDED(s->GetProperty(
					    SENSOR_PROPERTY_DEVICE_PATH, &pv)) &&
				    pv.vt == VT_LPWSTR && pv.pwszVal) {
					path = pv.pwszVal;
					for (auto &ch : path)
						ch = static_cast<wchar_t>(
							std::towlower(ch));
				}
				PropVariantClear(&pv);
				s->Release();
				if (path.empty())
					continue;
				for (size_t i = 0; i < g_device_registry.size();
				     ++i) {
					const device_entry &e =
						g_device_registry[i];
					if (e.profile.transport !=
					    imu_transport::sensor_api)
						continue;
					wchar_t vp[32];
					swprintf(vp, 32, L"vid_%04x&pid_%04x",
						 e.vid, e.pid);
					if (path.find(vp) !=
					    std::wstring::npos) {
						found = static_cast<model_id>(
							i + 1);
						if (out_present &&
						    out_present->size() < 400) {
							char id[32];
							snprintf(id, sizeof(id),
								 "sensorapi:%04X:%04X ",
								 e.vid, e.pid);
							*out_present += id;
						}
						break;
					}
				}
			}
			col->Release();
		}
		mgr->Release();
	}
	if (co_init)
		CoUninitialize();
	return found;
}

static model_id detect_hid_model(std::string *out_present = nullptr)
{
	model_id found = MODEL_UNKNOWN;
	for (const auto &info : enumerate_hid_interfaces()) {
		if (out_present && out_present->size() < 400) {
			char id[32];
			snprintf(id, sizeof(id), "%04X:%04X/%04X ",
				 info.vid, info.pid, info.usage_page);
			*out_present += id;
		}
		if (found == MODEL_UNKNOWN && info.model != MODEL_UNKNOWN)
			found = info.model;
	}
	// Fall back to the Sensor API for devices that don't surface as raw HID.
	if (found == MODEL_UNKNOWN)
		found = detect_sensor_api_model(out_present);
	return found;
}

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
};

static double clampd(double v, double lo, double hi)
{
	if (!std::isfinite(v))
		return lo;
	if (v < lo)
		return lo;
	if (v > hi)
		return hi;
	return v;
}

static double wrap_angle(double a)
{
	while (a > PI)
		a -= 2.0 * PI;
	while (a < -PI)
		a += 2.0 * PI;
	return a;
}

static quatd quat_normalize(quatd q)
{
	const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (!std::isfinite(n) || n <= 1e-12)
		return {};
	q.w /= n;
	q.x /= n;
	q.y /= n;
	q.z /= n;
	return q;
}

static quatd quat_inverse(quatd q)
{
	const double n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
	if (!std::isfinite(n2) || n2 <= 1e-12)
		return {};
	return {q.w / n2, -q.x / n2, -q.y / n2, -q.z / n2};
}

static quatd quat_multiply(quatd a, quatd b)
{
	return {
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	};
}

static quatd quat_from_rot_x(double rad)
{
	const double h = 0.5 * rad;
	return {std::cos(h), std::sin(h), 0.0, 0.0};
}

static quatd quat_from_yaw_y(double rad)
{
	const double h = 0.5 * rad;
	return {std::cos(h), 0.0, std::sin(h), 0.0};
}

static quatd quat_from_rot_z(double rad)
{
	const double h = 0.5 * rad;
	return {std::cos(h), 0.0, 0.0, std::sin(h)};
}

static vec3d rotate_vector(quatd q, vec3d v)
{
	const double tx = 2.0 * (q.y * v.z - q.z * v.y);
	const double ty = 2.0 * (q.z * v.x - q.x * v.z);
	const double tz = 2.0 * (q.x * v.y - q.y * v.x);
	return {
		v.x + q.w * tx + (q.y * tz - q.z * ty),
		v.y + q.w * ty + (q.z * tx - q.x * tz),
		v.z + q.w * tz + (q.x * ty - q.y * tx),
	};
}

static vec3d rotate_world_vector_into_body(quatd q, vec3d world)
{
	return rotate_vector(quat_normalize(quat_inverse(q)), world);
}

static quatd quat_derivative(quatd q, double wx, double wy, double wz)
{
	const quatd m = quat_multiply(q, {0.0, wx, wy, wz});
	return {0.5 * m.w, 0.5 * m.x, 0.5 * m.y, 0.5 * m.z};
}

static double yaw_from_quat_heading(quatd q, double fallback = 0.0)
{
	const vec3d f = rotate_vector(quat_normalize(q), {0.0, 0.0, -1.0});
	const double hn = std::sqrt(f.x * f.x + f.z * f.z);
	if (!std::isfinite(hn) || hn < 1e-6)
		return fallback;
	return wrap_angle(std::atan2(-(f.x / hn), -(f.z / hn)));
}

static uint32_t elapsed_us32(uint32_t now, uint32_t then)
{
	return static_cast<uint32_t>(now - then);
}

static uint16_t read_u16_le(const uint8_t *p)
{
	return static_cast<uint16_t>(p[0]) |
	       static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8);
}

static uint32_t read_u32_le(const uint8_t *p)
{
	return static_cast<uint32_t>(p[0]) |
	       (static_cast<uint32_t>(p[1]) << 8) |
	       (static_cast<uint32_t>(p[2]) << 16) |
	       (static_cast<uint32_t>(p[3]) << 24);
}

static uint64_t read_u64_le(const uint8_t *p)
{
	uint64_t v = 0;
	for (int i = 7; i >= 0; --i)
		v = (v << 8) | p[i];
	return v;
}

static uint32_t read_u24_le(const uint8_t *p)
{
	return static_cast<uint32_t>(p[0]) |
	       (static_cast<uint32_t>(p[1]) << 8) |
	       (static_cast<uint32_t>(p[2]) << 16);
}

static int16_t read_i16_le(const uint8_t *p)
{
	return static_cast<int16_t>(read_u16_le(p));
}

static int32_t read_i32_le(const uint8_t *p)
{
	return static_cast<int32_t>(read_u32_le(p));
}

static int16_t read_i16_be(const uint8_t *p)
{
	const uint16_t u = static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) |
						static_cast<uint16_t>(p[1]));
	return static_cast<int16_t>(u);
}

static int32_t read_i32_be(const uint8_t *p)
{
	const uint32_t u = (static_cast<uint32_t>(p[0]) << 24) |
			   (static_cast<uint32_t>(p[1]) << 16) |
			   (static_cast<uint32_t>(p[2]) << 8) |
			   static_cast<uint32_t>(p[3]);
	return static_cast<int32_t>(u);
}

static int32_t read_i24_le(const uint8_t *p)
{
	int32_t v = static_cast<int32_t>(read_u24_le(p));
	if (v & 0x800000)
		v -= 0x1000000;
	return v;
}

static int16_t read_air_mag_i16(const uint8_t *p)
{
	const uint16_t u = static_cast<uint16_t>(p[0]) |
			   static_cast<uint16_t>((p[1] ^ 0x80) << 8);
	return static_cast<int16_t>(u);
}

static float read_f32_le(const uint8_t *p)
{
	uint32_t u = read_u32_le(p);
	float f = 0.0f;
	std::memcpy(&f, &u, sizeof(f));
	return f;
}

static float read_f32_be(const uint8_t *p)
{
	const uint8_t b[4] = {p[3], p[2], p[1], p[0]};
	float f = 0.0f;
	std::memcpy(&f, b, sizeof(f));
	return f;
}

struct decoded_record {
	uint32_t kind = 0;
	imu_sample imu;
	mag_sample mag;
};

static bool decode_record(const uint8_t *rec, decoded_record &out)
{
	if (std::memcmp(rec, RAW_HDR, RAW_HDR_LEN) != 0)
		return false;
	const uint32_t kind = read_u32_le(rec + 30);
	if (kind != KIND_IMU && kind != KIND_MAG)
		return false;

	const uint64_t ts_raw = read_u64_le(rec + 14);
	const uint32_t ts_us = static_cast<uint32_t>(ts_raw / 1000ULL);
	const uint32_t seq = read_u24_le(rec + 75);

	out.kind = kind;
	if (kind == KIND_IMU) {
		out.imu.ts_us = ts_us;
		out.imu.seq = seq;
		out.imu.gx = read_f32_le(rec + 34);
		out.imu.gy = read_f32_le(rec + 38);
		out.imu.gz = read_f32_le(rec + 42);
		out.imu.ax = read_f32_le(rec + 46);
		out.imu.ay = read_f32_le(rec + 50);
		out.imu.az = read_f32_le(rec + 54);
	} else {
		out.mag.ts_us = ts_us;
		out.mag.seq = seq;
		out.mag.mx = read_f32_le(rec + 58);
		out.mag.my = read_f32_le(rec + 62);
		out.mag.mz = read_f32_le(rec + 66);
		out.mag.temp_c = read_f32_le(rec + 70);
	}
	return true;
}

struct air_timestamp_state {
	bool have_last = false;
	uint64_t last_sensor_ts = 0;
	uint32_t ts_us = 0;
};

// Shared by the Air and RayNeo HID decoders.
struct decoded_sensor_report {
	imu_sample imu;
	mag_sample mag;
	bool has_imu = false;
	bool has_mag = false;
};

static uint32_t now_us32()
{
	return static_cast<uint32_t>(os_gettime_ns() / 1000ULL);
}

static uint32_t air_ts_us32_from_report(uint64_t ts64, air_timestamp_state &state)
{
	if (!state.have_last) {
		state.have_last = true;
		state.last_sensor_ts = ts64;
		state.ts_us = now_us32();
		return state.ts_us;
	}

	const uint64_t delta = ts64 >= state.last_sensor_ts ? ts64 - state.last_sensor_ts : 0;
	state.last_sensor_ts = ts64;
	const uint64_t delta_us = delta / 1000ULL;
	if (delta_us > 500000ULL) {
		state.ts_us = now_us32();
		return state.ts_us;
	}
	state.ts_us += static_cast<uint32_t>(delta_us);
	return state.ts_us;
}

static bool decode_air_report(const uint8_t *data, size_t len,
			      air_timestamp_state &ts_state,
			      decoded_sensor_report &out)
{
	if (!data || len < 64)
		return false;
	const uint16_t signature = read_u16_le(data);
	if (signature == 0x53AA)
		return false;
	if (signature != 0x0201)
		return false;

	const uint32_t ts_us = air_ts_us32_from_report(read_u64_le(data + 4), ts_state);
	const int16_t gyro_mul = read_i16_le(data + 12);
	const int32_t gyro_div = read_i32_le(data + 14);
	const int16_t acc_mul = read_i16_le(data + 27);
	const int32_t acc_div = read_i32_le(data + 29);
	if (gyro_div == 0 || acc_div == 0)
		return false;

	const double deg_to_rad = PI / 180.0;
	const double gx0 =
		(static_cast<double>(read_i24_le(data + 18)) * gyro_mul / gyro_div) *
		deg_to_rad;
	const double gy0 =
		(static_cast<double>(read_i24_le(data + 21)) * gyro_mul / gyro_div) *
		deg_to_rad;
	const double gz0 =
		(static_cast<double>(read_i24_le(data + 24)) * gyro_mul / gyro_div) *
		deg_to_rad;
	const double ax0 = static_cast<double>(read_i24_le(data + 33)) * acc_mul /
			   acc_div;
	const double ay0 = static_cast<double>(read_i24_le(data + 36)) * acc_mul /
			   acc_div;
	const double az0 = static_cast<double>(read_i24_le(data + 39)) * acc_mul /
			   acc_div;

	const double t_gx = gy0;
	const double t_gy = gx0;
	const double t_gz = gz0;
	const double t_ax = ay0;
	const double t_ay = ax0;
	const double t_az = az0;

	out.imu.ts_us = ts_us;
	out.imu.gx = static_cast<float>(-t_gy);
	out.imu.gy = static_cast<float>(+t_gz);
	out.imu.gz = static_cast<float>(+t_gx);
	out.imu.ax = static_cast<float>(-t_ay);
	out.imu.ay = static_cast<float>(+t_az);
	out.imu.az = static_cast<float>(+t_ax);
	out.has_imu = true;

	const int16_t mag_mul = read_i16_be(data + 42);
	const int32_t mag_div = read_i32_be(data + 44);
	if (mag_div != 0) {
		const double mx0 =
			static_cast<double>(read_air_mag_i16(data + 48)) * mag_mul /
			mag_div;
		const double my0 =
			static_cast<double>(read_air_mag_i16(data + 50)) * mag_mul /
			mag_div;
		const double mz0 =
			static_cast<double>(read_air_mag_i16(data + 52)) * mag_mul /
			mag_div;
		const double t_mx = my0;
		const double t_my = mx0;
		const double t_mz = mz0;
		out.mag.ts_us = ts_us;
		out.mag.mx = static_cast<float>(+t_my);
		out.mag.my = static_cast<float>(-t_mz);
		out.mag.mz = static_cast<float>(-t_mx);
		out.mag.temp_c = 0.0f;
		out.has_mag = true;
	}

	return out.has_imu || out.has_mag;
}

// RayNeo HID input reports carry a byte stream of [0x99, kind, len, ...]
// frames that may split across reports; scan for the magic byte and
// reassemble, mirroring the packet assembler in rayneo_driver.js. Feeding the
// raw Windows report buffer (including a leading report-id byte) is fine
// because the scan skips everything before the magic.
class rayneo_packet_assembler {
public:
	void append(const uint8_t *data, size_t len)
	{
		if (!data || len == 0)
			return;
		if (stash_.size() + len > 4096)
			stash_.clear();
		stash_.insert(stash_.end(), data, data + len);
	}

	bool next_packet(std::vector<uint8_t> &out)
	{
		size_t offset = 0;
		while (offset + 4 <= stash_.size()) {
			if (stash_[offset] != RAYNEO_PKT_MAGIC) {
				offset++;
				continue;
			}
			const size_t packet_len = stash_[offset + 2];
			if (packet_len < 4) {
				offset++;
				continue;
			}
			if (offset + packet_len > stash_.size())
				break; // partial packet; keep from the magic
			out.assign(stash_.begin() + static_cast<ptrdiff_t>(offset),
				   stash_.begin() +
					   static_cast<ptrdiff_t>(offset + packet_len));
			consume(offset + packet_len);
			return true;
		}
		consume(offset);
		return false;
	}

private:
	void consume(size_t n)
	{
		if (n == 0)
			return;
		if (n >= stash_.size())
			stash_.clear();
		else
			stash_.erase(stash_.begin(),
				     stash_.begin() + static_cast<ptrdiff_t>(n));
	}

	std::vector<uint8_t> stash_;
};

// Mirrors parseRayNeoSensor in rayneo_driver.js: float32 LE fields, gyro in
// deg/s, a 100 us tick counter at offset 40. Unlike the XREAL Air decoder the
// reference driver feeds the device axes straight to the tracker, so there is
// no axis remap here.
static bool decode_rayneo_sensor_packet(const uint8_t *pkt, size_t len,
					decoded_sensor_report &out)
{
	out = {};
	if (!pkt || len < 56 || pkt[0] != RAYNEO_PKT_MAGIC ||
	    pkt[1] != RAYNEO_PKT_SENSOR)
		return false;

	const uint32_t ts_us = read_u32_le(pkt + 40) * 100u;
	const double deg_to_rad = PI / 180.0;
	out.imu.ts_us = ts_us;
	out.imu.gx = static_cast<float>(read_f32_le(pkt + 16) * deg_to_rad);
	out.imu.gy = static_cast<float>(read_f32_le(pkt + 20) * deg_to_rad);
	out.imu.gz = static_cast<float>(read_f32_le(pkt + 24) * deg_to_rad);
	out.imu.ax = read_f32_le(pkt + 4);
	out.imu.ay = read_f32_le(pkt + 8);
	out.imu.az = read_f32_le(pkt + 12);
	out.has_imu = true;

	out.mag.ts_us = ts_us;
	out.mag.mx = read_f32_le(pkt + 32);
	out.mag.my = read_f32_le(pkt + 36);
	out.mag.mz = read_f32_le(pkt + 52);
	out.mag.temp_c = read_f32_le(pkt + 28);
	out.has_mag = true;
	return true;
}

// Pairing state for Rokid type-4 packets, which deliver accel/gyro/mag as
// separate packets that share a timestamp base.
struct rokid_decode_state {
	vec3d accel;
	bool have_accel = false;
};

// Rokid Max / Air stream fixed 64-byte packets continuously with no start
// command. Layout facts from the openly documented protocol (see the
// ar-drivers-rs project's Rokid driver): type 4 = single sensor (byte 1:
// 1=accel, 2=gyro, 3=mag; u64 microsecond timestamp at offset 9; f32[3]
// vector at offset 21), type 17 = combined accel+gyro+mag (u64 nanosecond
// timestamp at offset 1; accel @9, gyro @21, mag @33). Values are SI
// (rad/s, m/s^2) in the shared X-right / Y-up / Z-toward-wearer frame, so
// there is no axis remap and no deg-to-rad conversion. Windows prepends a
// zero report-id byte for unnumbered HID reports; skip it when present.
static bool decode_rokid_packet(const uint8_t *data, size_t len,
				rokid_decode_state &st,
				decoded_sensor_report &out)
{
	if (!data || len < 2)
		return false;
	if (data[0] == 0 &&
	    (data[1] == ROKID_PKT_SENSOR || data[1] == ROKID_PKT_COMBINED)) {
		data++;
		len--;
	}
	out = {};
	if (data[0] == ROKID_PKT_SENSOR && len >= 33) {
		const uint8_t sensor_type = data[1];
		const uint64_t ts_us = read_u64_le(data + 9);
		const double v0 = read_f32_le(data + 21);
		const double v1 = read_f32_le(data + 25);
		const double v2 = read_f32_le(data + 29);
		switch (sensor_type) {
		case 1: // accelerometer: cache until the paired gyro arrives
			st.accel = {v0, v1, v2};
			st.have_accel = true;
			return false;
		case 2: // gyroscope: emit together with the latest accel
			if (!st.have_accel)
				return false;
			out.imu.gx = static_cast<float>(v0);
			out.imu.gy = static_cast<float>(v1);
			out.imu.gz = static_cast<float>(v2);
			out.imu.ax = static_cast<float>(st.accel.x);
			out.imu.ay = static_cast<float>(st.accel.y);
			out.imu.az = static_cast<float>(st.accel.z);
			out.imu.ts_us = static_cast<uint32_t>(ts_us);
			out.has_imu = true;
			return true;
		case 3: // magnetometer
			out.mag.mx = static_cast<float>(v0);
			out.mag.my = static_cast<float>(v1);
			out.mag.mz = static_cast<float>(v2);
			out.mag.temp_c = 0.0f;
			out.mag.ts_us = static_cast<uint32_t>(ts_us);
			out.has_mag = true;
			return true;
		default:
			return false;
		}
	}
	if (data[0] == ROKID_PKT_COMBINED && len >= 45) {
		const uint64_t ts_us = read_u64_le(data + 1) / 1000;
		out.imu.ax = read_f32_le(data + 9);
		out.imu.ay = read_f32_le(data + 13);
		out.imu.az = read_f32_le(data + 17);
		out.imu.gx = read_f32_le(data + 21);
		out.imu.gy = read_f32_le(data + 25);
		out.imu.gz = read_f32_le(data + 29);
		out.imu.ts_us = static_cast<uint32_t>(ts_us);
		out.has_imu = true;
		out.mag.mx = read_f32_le(data + 33);
		out.mag.my = read_f32_le(data + 37);
		out.mag.mz = read_f32_le(data + 41);
		out.mag.temp_c = 0.0f;
		out.mag.ts_us = static_cast<uint32_t>(ts_us);
		out.has_mag = true;
		return true;
	}
	return false;
}

// Mirrors parseRayNeoDeviceInfo in rayneo_driver.js: the response to
// ACQUIRE_DEVICE_INFO carries the board id, which decides the IMU mount
// offset (boardId 0x3A = -20 deg, otherwise 0 deg).
static bool decode_rayneo_device_info(const uint8_t *pkt, size_t len,
				      int &board_id)
{
	if (!pkt || len <= 43 || pkt[0] != RAYNEO_PKT_MAGIC ||
	    pkt[1] != RAYNEO_PKT_RESPONSE ||
	    pkt[8] != RAYNEO_CMD_ACQUIRE_DEVICE_INFO)
		return false;
	board_id = pkt[21];
	return true;
}

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
		return {qrel, latest_omega_, last_imu_ts_us_, is_calibrated_,
			false, bias_auto_active_, imu_count_, mag_count_};
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
				blog(LOG_INFO,
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

class winsock_scope {
public:
	winsock_scope()
	{
		WSADATA wsa = {};
		ok_ = WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
	}
	~winsock_scope()
	{
		if (ok_)
			WSACleanup();
	}
	bool ok() const { return ok_; }

private:
	bool ok_ = false;
};

static bool find_local_bind_ip(std::string &ip)
{
	ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
		      GAA_FLAG_SKIP_DNS_SERVER;
	ULONG family = AF_INET;
	ULONG size = 0;
	if (GetAdaptersAddresses(family, flags, nullptr, nullptr, &size) !=
	    ERROR_BUFFER_OVERFLOW)
		return false;

	std::vector<uint8_t> buf(size);
	auto *addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buf.data());
	if (GetAdaptersAddresses(family, flags, nullptr, addrs, &size) != NO_ERROR)
		return false;

	for (auto *adapter = addrs; adapter; adapter = adapter->Next) {
		if (adapter->OperStatus != IfOperStatusUp)
			continue;
		for (auto *ua = adapter->FirstUnicastAddress; ua; ua = ua->Next) {
			auto *sa = reinterpret_cast<sockaddr_in *>(ua->Address.lpSockaddr);
			if (!sa || sa->sin_family != AF_INET)
				continue;
			char text[INET_ADDRSTRLEN] = {};
			if (!inet_ntop(AF_INET, &sa->sin_addr, text, sizeof(text)))
				continue;
			std::string candidate(text);
			if (candidate.rfind("169.254.2.", 0) == 0) {
				ip = candidate;
				return true;
			}
		}
	}
	return false;
}

static SOCKET connect_tcp(const std::string &host, int port, int timeout_ms,
			  std::string &bind_used)
{
	SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (s == INVALID_SOCKET)
		return INVALID_SOCKET;

	std::string bind_ip;
	if (find_local_bind_ip(bind_ip)) {
		sockaddr_in local = {};
		local.sin_family = AF_INET;
		local.sin_port = 0;
		inet_pton(AF_INET, bind_ip.c_str(), &local.sin_addr);
		if (bind(s, reinterpret_cast<sockaddr *>(&local), sizeof(local)) == 0)
			bind_used = bind_ip;
	}

	u_long nonblock = 1;
	ioctlsocket(s, FIONBIO, &nonblock);

	sockaddr_in addr = {};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(static_cast<u_short>(port));
	if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
		closesocket(s);
		return INVALID_SOCKET;
	}

	const int rc = connect(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
	if (rc == SOCKET_ERROR) {
		const int err = WSAGetLastError();
		if (err != WSAEWOULDBLOCK && err != WSAEINPROGRESS) {
			closesocket(s);
			return INVALID_SOCKET;
		}
		fd_set write_set;
		FD_ZERO(&write_set);
		FD_SET(s, &write_set);
		timeval tv = {};
		tv.tv_sec = timeout_ms / 1000;
		tv.tv_usec = (timeout_ms % 1000) * 1000;
		if (select(0, nullptr, &write_set, nullptr, &tv) <= 0) {
			closesocket(s);
			return INVALID_SOCKET;
		}
		int so_error = 0;
		int len = sizeof(so_error);
		getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char *>(&so_error),
			   &len);
		if (so_error != 0) {
			closesocket(s);
			return INVALID_SOCKET;
		}
	}

	nonblock = 0;
	ioctlsocket(s, FIONBIO, &nonblock);
	BOOL nodelay = TRUE;
	setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&nodelay),
		   sizeof(nodelay));
	DWORD recv_timeout = 1000;
	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO,
		   reinterpret_cast<const char *>(&recv_timeout), sizeof(recv_timeout));
	return s;
}

static uint32_t air_crc32(const uint8_t *data, size_t len)
{
	static uint32_t table[256] = {};
	static bool table_ready = false;
	if (!table_ready) {
		for (uint32_t i = 0; i < 256; ++i) {
			uint32_t c = i;
			for (int k = 0; k < 8; ++k)
				c = (c & 1) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
			table[i] = c;
		}
		table_ready = true;
	}

	uint32_t c = 0xFFFFFFFFU;
	for (size_t i = 0; i < len; ++i)
		c = table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
	return c ^ 0xFFFFFFFFU;
}

static std::vector<uint8_t> build_air_packet(uint8_t msg_id,
					     const std::vector<uint8_t> &payload = {})
{
	const uint16_t packet_len = static_cast<uint16_t>(3 + payload.size());
	std::vector<uint8_t> chk(3 + payload.size());
	chk[0] = static_cast<uint8_t>(packet_len & 0xFF);
	chk[1] = static_cast<uint8_t>((packet_len >> 8) & 0xFF);
	chk[2] = msg_id;
	if (!payload.empty())
		std::memcpy(chk.data() + 3, payload.data(), payload.size());

	const uint32_t checksum = air_crc32(chk.data(), chk.size());
	std::vector<uint8_t> packet(1 + 4 + chk.size());
	packet[0] = 0xAA;
	packet[1] = static_cast<uint8_t>(checksum & 0xFF);
	packet[2] = static_cast<uint8_t>((checksum >> 8) & 0xFF);
	packet[3] = static_cast<uint8_t>((checksum >> 16) & 0xFF);
	packet[4] = static_cast<uint8_t>((checksum >> 24) & 0xFF);
	std::memcpy(packet.data() + 5, chk.data(), chk.size());
	return packet;
}

static bool wait_overlapped_result(HANDLE h, OVERLAPPED &ov, DWORD timeout_ms,
				   DWORD &bytes)
{
	const DWORD wait = WaitForSingleObject(ov.hEvent, timeout_ms);
	if (wait == WAIT_OBJECT_0)
		return GetOverlappedResult(h, &ov, &bytes, FALSE) != FALSE;
	if (wait == WAIT_TIMEOUT) {
		CancelIoEx(h, &ov);
		WaitForSingleObject(ov.hEvent, 50);
	}
	return false;
}

static bool hid_write_report(HANDLE h, USHORT output_len,
			     const std::vector<uint8_t> &payload, DWORD timeout_ms)
{
	const size_t report_len =
		std::max<size_t>(output_len ? output_len : 0, payload.size() + 1);
	std::vector<uint8_t> report(report_len, 0);
	std::memcpy(report.data() + 1, payload.data(), payload.size());

	OVERLAPPED ov = {};
	ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
	if (!ov.hEvent)
		return false;
	DWORD bytes = 0;
	BOOL ok = WriteFile(h, report.data(), static_cast<DWORD>(report.size()), nullptr,
			    &ov);
	if (!ok && GetLastError() == ERROR_IO_PENDING)
		ok = wait_overlapped_result(h, ov, timeout_ms, bytes);
	else if (ok)
		ok = GetOverlappedResult(h, &ov, &bytes, FALSE);
	CloseHandle(ov.hEvent);
	return ok != FALSE;
}

static bool hid_read_report(HANDLE h, USHORT input_len, std::vector<uint8_t> &data,
			    DWORD timeout_ms)
{
	const size_t report_len = std::max<size_t>(input_len ? input_len : 0, 65);
	std::vector<uint8_t> report(report_len, 0);

	OVERLAPPED ov = {};
	ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
	if (!ov.hEvent)
		return false;
	DWORD bytes = 0;
	BOOL ok = ReadFile(h, report.data(), static_cast<DWORD>(report.size()), nullptr,
			   &ov);
	if (!ok && GetLastError() == ERROR_IO_PENDING)
		ok = wait_overlapped_result(h, ov, timeout_ms, bytes);
	else if (ok)
		ok = GetOverlappedResult(h, &ov, &bytes, FALSE);
	CloseHandle(ov.hEvent);
	if (!ok || bytes == 0)
		return false;

	size_t offset = 0;
	if (bytes > 2 && report[0] == 0) {
		const uint16_t sig_at_one = read_u16_le(report.data() + 1);
		if (sig_at_one == 0x0201 || sig_at_one == 0x53AA || report[1] == 0xAA)
			offset = 1;
	}
	data.assign(report.begin() + static_cast<ptrdiff_t>(offset),
		    report.begin() + static_cast<ptrdiff_t>(bytes));
	return true;
}

static bool air_send_packet(HANDLE h, const hid_interface_info &info,
			    uint8_t msg_id,
			    const std::vector<uint8_t> &payload = {})
{
	return hid_write_report(h, info.output_report_len,
				build_air_packet(msg_id, payload), 250);
}

static HANDLE open_hid_path_rw(const std::wstring &path)
{
	return CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE,
			   FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
			   FILE_FLAG_OVERLAPPED, nullptr);
}

struct device_manager {
	std::mutex state_mutex;
	head_tracker tracker;
	pose_snapshot pose;

	std::thread worker;
	std::atomic<bool> stop{false};
	std::atomic<uint32_t> reconnect_epoch{0};
	std::atomic<bool> connected{false};
	std::atomic<bool> connect_enabled{true};
	std::atomic<bool> debug_log{false};
	std::atomic<bool> mag_yaw{false};
	std::atomic<bool> auto_projector{false};
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
	std::atomic<bool> fov_auto{true};
	std::atomic<int> virtual_source_count{0};
	std::atomic<float> prediction_ms{10.0f};
	std::atomic<float> fov_deg{50.0f};
	std::atomic<float> screen_distance_m{DEFAULT_SCREEN_DISTANCE_M};
	std::atomic<float> screen_size_factor{1.0f};
	std::atomic<float> screen_curve{DEFAULT_SCREEN_CURVE};

	std::mutex settings_mutex;
	std::string ip = "169.254.2.1";
	int port = 52998;
};

struct nyan_real_virtual_source {
	obs_source_t *context = nullptr;
	obs_source_t *target = nullptr;
	gs_texrender_t *texrender = nullptr;
	gs_effect_t *effect = nullptr;
	gs_eparam_t *p_image = nullptr;
	gs_eparam_t *p_pose_q = nullptr;
	gs_eparam_t *p_pose_valid = nullptr;
	gs_eparam_t *p_tan_half_fov = nullptr;
	gs_eparam_t *p_screen_distance_m = nullptr;
	gs_eparam_t *p_screen_half_size_m = nullptr;
	gs_eparam_t *p_screen_curve = nullptr;
	gs_eparam_t *p_debug_tint = nullptr;
	std::string target_name;
	uint32_t output_width = 1920;
	uint32_t output_height = 1080;
	bool target_active_child = false;
	bool target_recursion_blocked = false;
	// One target capture per output frame: video_render runs once per view
	// (preview, program, projectors), but re-rendering the target into the
	// texrender for each of them only repeats identical work. Reset by
	// video_tick, set after a successful capture.
	bool captured_this_frame = false;
	float target_retry_timer_s = 0.0f;
	uint64_t last_render_log_ns = 0;
};

static device_manager g_device;

// Actual mode of the EDID-identified glasses display, cached by the dock's
// poll so the video thread never touches the Win32 display APIs. 0 = no
// glasses display present; virtual sources then fall back to the HID-detected
// device's profile resolution.
static std::atomic<uint32_t> g_glasses_display_width{0};
static std::atomic<uint32_t> g_glasses_display_height{0};

static void publish_pose(device_manager *f, bool connected)
{
	std::lock_guard<std::mutex> lk(f->state_mutex);
	f->pose = f->tracker.snapshot();
	f->pose.connected = connected;
}

static model_id detected_hid_model(const device_manager *f)
{
	return f->detected_model.load(std::memory_order_relaxed);
}

static bool hid_device_ready(const device_manager *f)
{
	return detected_hid_model(f) != MODEL_UNKNOWN;
}

static imu_transport detected_transport_for(const device_manager *f)
{
	return profile_for(detected_hid_model(f)).transport;
}

static bool publish_sensor_samples(device_manager *f, const imu_sample *imu,
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
static bool publish_external_pose(device_manager *f, const quatd &device_q,
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

// Re-run HID detection at most once per second and, on a model change, update
// the detected model and (in auto-detect mode with auto FOV) the FOV. Called
// from both the idle/reconnect loop and the connected recv loop so detection
// keeps working while the IMU stream is open.
static void refresh_detected_model(device_manager *f, uint64_t &last_detect_ns)
{
	const uint64_t now_ns = os_gettime_ns();
	if (last_detect_ns != 0 && now_ns - last_detect_ns < 1000000000ULL)
		return;
	last_detect_ns = now_ns;

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

// Returns true when at least one valid record was decoded, so the caller can
// distinguish live IMU data from a connection that only delivers garbage.
static bool consume_stream_bytes(device_manager *f, std::vector<uint8_t> &stash,
				 const uint8_t *buf, size_t len)
{
	bool decoded_any = false;
	if (!hid_device_ready(f)) {
		stash.clear();
		return false;
	}

	stash.insert(stash.end(), buf, buf + len);
	while (true) {
		if (stash.size() < RAW_HDR_LEN)
			return decoded_any;
		size_t hdr = std::string::npos;
		for (size_t i = 0; i + RAW_HDR_LEN <= stash.size(); ++i) {
			if (std::memcmp(stash.data() + i, RAW_HDR, RAW_HDR_LEN) == 0) {
				hdr = i;
				break;
			}
		}
		if (hdr == std::string::npos) {
			const size_t keep = std::min(stash.size(), RAW_HDR_LEN - 1);
			std::vector<uint8_t> tail(stash.end() - keep, stash.end());
			stash.swap(tail);
			return decoded_any;
		}
		if (hdr > 0)
			stash.erase(stash.begin(), stash.begin() + static_cast<ptrdiff_t>(hdr));
		if (stash.size() < RAW_RECORD_LEN)
			return decoded_any;

		decoded_record rec;
		if (decode_record(stash.data(), rec)) {
			if (rec.kind == KIND_IMU)
				publish_sensor_samples(f, &rec.imu, nullptr);
			else
				publish_sensor_samples(f, nullptr, &rec.mag);
			decoded_any = true;
		}
		stash.erase(stash.begin(), stash.begin() + static_cast<ptrdiff_t>(RAW_RECORD_LEN));
	}
}

struct rate_log_state {
	uint64_t last_log_ns = 0;
	uint64_t last_imu = 0;
	uint64_t last_mag = 0;
	quatd last_q;
	bool have_q = false;
};

static void maybe_log_sensor_rate(device_manager *f, rate_log_state &st,
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

static void run_one_bridge_tcp_session(device_manager *f, uint32_t &seen_epoch,
				       uint64_t &last_detect_ns)
{
	winsock_scope ws;
	if (!ws.ok()) {
		blog(LOG_ERROR, "[obs-nyan-real-3dof] WSAStartup failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	std::string ip;
	int port = 0;
	{
		std::lock_guard<std::mutex> lk(f->settings_mutex);
		ip = f->ip;
		port = f->port;
	}

	std::string bind_used;
	SOCKET s = connect_tcp(ip, port, 1500, bind_used);
	if (s == INVALID_SOCKET) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING, "[obs-nyan-real-3dof] TCP connect failed: %s:%d",
			     ip.c_str(), port);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed)) {
		blog(LOG_INFO, "[obs-nyan-real-3dof] TCP connected: %s:%d%s%s",
		     ip.c_str(), port, bind_used.empty() ? "" : " bind=",
		     bind_used.empty() ? "" : bind_used.c_str());
	}

	std::vector<uint8_t> stash;
	stash.reserve(65536);
	std::vector<uint8_t> buf(16384);
	rate_log_state rate_log;
	uint64_t last_record_ns = os_gettime_ns();

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		refresh_detected_model(f, last_detect_ns);
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::one_bridge_tcp)
			break;
		// A TCP connection can stay open while delivering nothing (or
		// garbage). Tear the session down so the outer loop reconnects;
		// this is the TCP counterpart of the Air/RayNeo in-session
		// retries and makes a manual reconnect button unnecessary.
		if (os_gettime_ns() - last_record_ns > 3000000000ULL) {
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] TCP stream stalled "
				     "(no valid records for 3 s); reconnecting");
			break;
		}
		const int n = recv(s, reinterpret_cast<char *>(buf.data()),
				   static_cast<int>(buf.size()), 0);
		if (n > 0) {
			if (consume_stream_bytes(f, stash, buf.data(),
						 static_cast<size_t>(n)))
				last_record_ns = os_gettime_ns();
			maybe_log_sensor_rate(f, rate_log, "TCP");
			continue;
		}
		if (n == 0)
			break;
		const int err = WSAGetLastError();
		if (err == WSAETIMEDOUT || err == WSAEWOULDBLOCK)
			continue;
		break;
	}

	closesocket(s);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] TCP disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

static bool looks_like_air_report(const std::vector<uint8_t> &data)
{
	if (data.empty())
		return false;
	if (data[0] == 0xAA)
		return true;
	if (data.size() >= 2) {
		const uint16_t sig = read_u16_le(data.data());
		return sig == 0x0201 || sig == 0x53AA;
	}
	return false;
}

static HANDLE open_air_hid_stream(hid_interface_info &selected)
{
	for (const auto &info : enumerate_hid_interfaces()) {
		if (profile_for(info.model).transport != imu_transport::air_hid ||
		    is_consumer_control_hid(info))
			continue;
		HANDLE h = open_hid_path_rw(info.path);
		if (h == INVALID_HANDLE_VALUE)
			continue;

		bool ok = false;
		air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x00});
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		air_send_packet(h, info, AIR_MSG_GET_STATIC_ID);
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x01});

		const uint64_t start = os_gettime_ns();
		while (os_gettime_ns() - start < 800000000ULL) {
			std::vector<uint8_t> report;
			if (hid_read_report(h, info.input_report_len, report, 100) &&
			    looks_like_air_report(report)) {
				ok = true;
				break;
			}
		}

		if (ok) {
			selected = info;
			return h;
		}
		air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x00});
		CloseHandle(h);
	}
	return INVALID_HANDLE_VALUE;
}

static void run_air_hid_session(device_manager *f, uint32_t &seen_epoch,
				uint64_t &last_detect_ns)
{
	hid_interface_info info;
	HANDLE h = open_air_hid_stream(info);
	if (h == INVALID_HANDLE_VALUE) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] Air HID connect failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO, "[obs-nyan-real-3dof] Air HID connected");

	air_timestamp_state ts_state;
	uint32_t seq = 0;
	uint64_t last_rx_ns = os_gettime_ns();
	uint64_t last_start_retry_ns = last_rx_ns;
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		refresh_detected_model(f, last_detect_ns);
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::air_hid)
			break;

		std::vector<uint8_t> report;
		if (!hid_read_report(h, info.input_report_len, report, 250)) {
			const uint64_t now = os_gettime_ns();
			if (now - last_rx_ns > 1200000000ULL &&
			    now - last_start_retry_ns > 1200000000ULL) {
				last_start_retry_ns = now;
				air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x00});
				std::this_thread::sleep_for(std::chrono::milliseconds(80));
				air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x01});
			}
			continue;
		}
		decoded_sensor_report decoded;
		if (!decode_air_report(report.data(), report.size(), ts_state, decoded))
			continue;
		last_rx_ns = os_gettime_ns();
		decoded.imu.seq = seq;
		decoded.mag.seq = seq;
		seq++;
		publish_sensor_samples(f, decoded.has_imu ? &decoded.imu : nullptr,
				       decoded.has_mag ? &decoded.mag : nullptr);
		maybe_log_sensor_rate(f, rate_log, "Air HID");
	}

	air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x00});
	CloseHandle(h);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] Air HID disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

static bool rayneo_send_command(HANDLE h, const hid_interface_info &info,
				uint8_t cmd, uint8_t arg = 0)
{
	// The reference driver sends a full 64-byte report with report id 0.
	std::vector<uint8_t> payload(64, 0);
	payload[0] = RAYNEO_CMD_HEADER;
	payload[1] = cmd;
	payload[2] = arg;
	return hid_write_report(h, info.output_report_len, payload, 250);
}

// Probe every non-consumer-control HID interface of a rayneo_hid model:
// request device info, open the IMU, and accept the interface once a sensor
// packet assembles, mirroring probeDevice in rayneo_driver.js. The device-info
// response usually arrives during this probe, so the board id is captured here
// (board_id stays -1 when it was not seen).
static HANDLE open_rayneo_hid_stream(hid_interface_info &selected, int &board_id)
{
	board_id = -1;
	for (const auto &info : enumerate_hid_interfaces()) {
		if (profile_for(info.model).transport != imu_transport::rayneo_hid ||
		    is_consumer_control_hid(info))
			continue;
		HANDLE h = open_hid_path_rw(info.path);
		if (h == INVALID_HANDLE_VALUE)
			continue;

		rayneo_send_command(h, info, RAYNEO_CMD_ACQUIRE_DEVICE_INFO);
		std::this_thread::sleep_for(std::chrono::milliseconds(80));
		rayneo_send_command(h, info, RAYNEO_CMD_OPEN_IMU);

		bool ok = false;
		int probe_board_id = -1;
		rayneo_packet_assembler assembler;
		std::vector<uint8_t> report;
		std::vector<uint8_t> packet;
		decoded_sensor_report decoded;
		const uint64_t start = os_gettime_ns();
		while (!ok && os_gettime_ns() - start < 1200000000ULL) {
			if (!hid_read_report(h, info.input_report_len, report, 100))
				continue;
			assembler.append(report.data(), report.size());
			while (assembler.next_packet(packet)) {
				if (decode_rayneo_device_info(packet.data(),
							      packet.size(),
							      probe_board_id))
					continue;
				if (decode_rayneo_sensor_packet(packet.data(),
								packet.size(),
								decoded)) {
					ok = true;
					break;
				}
			}
		}

		if (ok) {
			selected = info;
			board_id = probe_board_id;
			return h;
		}
		rayneo_send_command(h, info, RAYNEO_CMD_CLOSE_IMU);
		CloseHandle(h);
	}
	return INVALID_HANDLE_VALUE;
}

// boardId 0x3A mounts the IMU at -20 deg; other boards sit level. Mirrors the
// reference driver's deviceInfo.mountXDeg handling, which overrides the static
// profile mount.
static void apply_rayneo_board_mount(device_manager *f, int board_id)
{
	const double deg = board_id == 0x3A ? -20.0 : 0.0;
	f->mount_override_cdeg.store(static_cast<int>(deg * 100.0),
				     std::memory_order_relaxed);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] RayNeo board id 0x%02X -> mount %+.0f deg",
		     board_id, deg);
}

static void run_rayneo_hid_session(device_manager *f, uint32_t &seen_epoch,
				   uint64_t &last_detect_ns)
{
	f->mount_override_cdeg.store(INT32_MIN, std::memory_order_relaxed);
	hid_interface_info info;
	int board_id = -1;
	HANDLE h = open_rayneo_hid_stream(info, board_id);
	if (h == INVALID_HANDLE_VALUE) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] RayNeo HID connect failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO, "[obs-nyan-real-3dof] RayNeo HID connected");
	if (board_id >= 0)
		apply_rayneo_board_mount(f, board_id);

	rayneo_packet_assembler assembler;
	std::vector<uint8_t> report;
	std::vector<uint8_t> packet;
	uint32_t seq = 0;
	uint64_t last_rx_ns = os_gettime_ns();
	uint64_t last_open_retry_ns = last_rx_ns;
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		refresh_detected_model(f, last_detect_ns);
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::rayneo_hid)
			break;

		if (!hid_read_report(h, info.input_report_len, report, 250)) {
			const uint64_t now = os_gettime_ns();
			if (now - last_rx_ns > 1200000000ULL &&
			    now - last_open_retry_ns > 1200000000ULL) {
				last_open_retry_ns = now;
				rayneo_send_command(h, info,
						    RAYNEO_CMD_ACQUIRE_DEVICE_INFO);
				std::this_thread::sleep_for(
					std::chrono::milliseconds(80));
				rayneo_send_command(h, info, RAYNEO_CMD_OPEN_IMU);
			}
			continue;
		}
		assembler.append(report.data(), report.size());
		while (assembler.next_packet(packet)) {
			int session_board_id = -1;
			if (decode_rayneo_device_info(packet.data(), packet.size(),
						      session_board_id)) {
				// Response to the stale-retry ACQUIRE, or one
				// the probe did not catch.
				apply_rayneo_board_mount(f, session_board_id);
				continue;
			}
			decoded_sensor_report decoded;
			if (!decode_rayneo_sensor_packet(packet.data(),
							 packet.size(), decoded))
				continue;
			last_rx_ns = os_gettime_ns();
			decoded.imu.seq = seq;
			decoded.mag.seq = seq;
			seq++;
			publish_sensor_samples(f,
					       decoded.has_imu ? &decoded.imu
							       : nullptr,
					       decoded.has_mag ? &decoded.mag
							       : nullptr);
			maybe_log_sensor_rate(f, rate_log, "RayNeo HID");
		}
	}

	rayneo_send_command(h, info, RAYNEO_CMD_CLOSE_IMU);
	CloseHandle(h);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] RayNeo HID disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

// --- Windows Sensor API transport (EPSON MOVERIO BT-40) ---------------------
// The BT-40 exposes its IMU as standard HID sensor collections, so the data is
// read through the Windows Sensor API instead of a custom protocol (the
// device's COM port only carries display-control commands such as setbright).
// Axes per the MOVERIO Basic Function SDK developer's guide: gyro/compass use
// X-right / Y-up / Z-toward-the-wearer, while the accelerometer alone is
// inverted (X-left / Y-down / Z-forward), so it is negated into the shared
// frame.

static bool propvariant_to_double(const PROPVARIANT &pv, double &out)
{
	switch (pv.vt) {
	case VT_R8:
		out = pv.dblVal;
		return true;
	case VT_R4:
		out = pv.fltVal;
		return true;
	case VT_I4:
		out = pv.lVal;
		return true;
	case VT_UI4:
		out = pv.ulVal;
		return true;
	default:
		return false;
	}
}

// Read a 3-axis value from the sensor's latest report. Returns true only when
// the report timestamp advanced (the API hands back the same report between
// hardware updates) and all three fields decoded; last_ft100 holds the report
// FILETIME in 100 ns units for timestamping the sample.
static bool sensor_read_vec3(ISensor *sensor, const PROPERTYKEY &kx,
			     const PROPERTYKEY &ky, const PROPERTYKEY &kz,
			     uint64_t &last_ft100, vec3d &out)
{
	ISensorDataReport *report = nullptr;
	if (FAILED(sensor->GetData(&report)) || !report)
		return false;

	SYSTEMTIME st = {};
	FILETIME ft = {};
	bool fresh = false;
	if (SUCCEEDED(report->GetTimestamp(&st)) &&
	    SystemTimeToFileTime(&st, &ft)) {
		const uint64_t t =
			(static_cast<uint64_t>(ft.dwHighDateTime) << 32) |
			ft.dwLowDateTime;
		fresh = t != last_ft100;
		if (fresh)
			last_ft100 = t;
	}

	bool ok = fresh;
	if (fresh) {
		const PROPERTYKEY keys[3] = {kx, ky, kz};
		double v[3] = {};
		for (int i = 0; i < 3 && ok; ++i) {
			PROPVARIANT pv;
			PropVariantInit(&pv);
			ok = SUCCEEDED(report->GetSensorValue(keys[i], &pv)) &&
			     propvariant_to_double(pv, v[i]);
			PropVariantClear(&pv);
		}
		if (ok)
			out = {v[0], v[1], v[2]};
	}
	report->Release();
	return ok;
}

static void sensor_request_min_interval(ISensor *sensor)
{
	PROPVARIANT pv;
	PropVariantInit(&pv);
	ULONG min_ms = 0;
	if (SUCCEEDED(sensor->GetProperty(SENSOR_PROPERTY_MIN_REPORT_INTERVAL,
					  &pv)) &&
	    pv.vt == VT_UI4)
		min_ms = pv.ulVal;
	PropVariantClear(&pv);

	IPortableDeviceValues *props = nullptr;
	if (FAILED(CoCreateInstance(CLSID_PortableDeviceValues, nullptr,
				    CLSCTX_INPROC_SERVER,
				    IID_PPV_ARGS(&props))))
		return;
	props->SetUnsignedIntegerValue(SENSOR_PROPERTY_CURRENT_REPORT_INTERVAL,
				       min_ms ? min_ms : 4);
	IPortableDeviceValues *results = nullptr;
	sensor->SetProperties(props, &results);
	if (results)
		results->Release();
	props->Release();
}

// ISensor::GetData can block indefinitely while the device sleeps (MOVERIO
// mutes its display and stops sensor reports after sitting still), which
// would also wedge the watchdog. Only call GetData when the sensor reports
// SENSOR_STATE_READY; GetState is a cheap non-blocking query.
static bool sensor_is_ready(ISensor *sensor)
{
	SensorState state = SENSOR_STATE_ERROR;
	return SUCCEEDED(sensor->GetState(&state)) &&
	       state == SENSOR_STATE_READY;
}

// Pick the sensor of the given type whose device path contains the detected
// USB identity, so a sensor on another device (laptop IMU etc.) is never used.
static ISensor *find_device_sensor(ISensorManager *mgr, REFSENSOR_TYPE_ID type,
				   const std::wstring &vidpid)
{
	ISensorCollection *col = nullptr;
	if (FAILED(mgr->GetSensorsByType(type, &col)) || !col)
		return nullptr;
	ULONG count = 0;
	col->GetCount(&count);
	ISensor *found = nullptr;
	for (ULONG i = 0; i < count && !found; ++i) {
		ISensor *s = nullptr;
		if (FAILED(col->GetAt(i, &s)) || !s)
			continue;
		PROPVARIANT pv;
		PropVariantInit(&pv);
		bool match = false;
		if (SUCCEEDED(s->GetProperty(SENSOR_PROPERTY_DEVICE_PATH, &pv)) &&
		    pv.vt == VT_LPWSTR && pv.pwszVal) {
			std::wstring path = pv.pwszVal;
			for (auto &c : path)
				c = static_cast<wchar_t>(std::towlower(c));
			match = path.find(vidpid) != std::wstring::npos;
		}
		PropVariantClear(&pv);
		if (match)
			found = s;
		else
			s->Release();
	}
	col->Release();
	return found;
}

// --- MOVERIO display-control serial port ------------------------------------
// The BT-40 family has no hardware brightness control; it accepts the text
// commands documented in the MOVERIO Basic Function SDK's Windows command
// reference over a USB-CDC COM port (setbright 0-20 / 50 = auto, getbright).

static bool find_device_com_port(uint16_t vid, uint16_t pid, std::wstring &out)
{
	HDEVINFO devs = SetupDiGetClassDevsW(&GUID_DEVINTERFACE_COMPORT, nullptr,
					     nullptr,
					     DIGCF_PRESENT |
						     DIGCF_DEVICEINTERFACE);
	if (devs == INVALID_HANDLE_VALUE)
		return false;
	wchar_t want[32];
	swprintf(want, 32, L"vid_%04x&pid_%04x", vid, pid);
	bool found = false;
	SP_DEVINFO_DATA dev = {};
	dev.cbSize = sizeof(dev);
	for (DWORD i = 0; !found && SetupDiEnumDeviceInfo(devs, i, &dev); ++i) {
		wchar_t inst[512] = L"";
		if (!SetupDiGetDeviceInstanceIdW(devs, &dev, inst, 512, nullptr))
			continue;
		for (wchar_t *p = inst; *p; ++p)
			*p = static_cast<wchar_t>(std::towlower(*p));
		if (!wcsstr(inst, want))
			continue;
		HKEY key = SetupDiOpenDevRegKey(devs, &dev, DICS_FLAG_GLOBAL, 0,
						DIREG_DEV, KEY_READ);
		if (key == INVALID_HANDLE_VALUE)
			continue;
		wchar_t port[64] = L"";
		DWORD len = sizeof(port);
		DWORD type = 0;
		if (RegQueryValueExW(key, L"PortName", nullptr, &type,
				     reinterpret_cast<LPBYTE>(port), &len) ==
			    ERROR_SUCCESS &&
		    type == REG_SZ && port[0]) {
			out = L"\\\\.\\" + std::wstring(port);
			found = true;
		}
		RegCloseKey(key);
	}
	SetupDiDestroyDeviceInfoList(devs);
	return found;
}

static HANDLE open_moverio_serial(uint16_t vid, uint16_t pid)
{
	std::wstring path;
	if (!find_device_com_port(vid, pid, path))
		return INVALID_HANDLE_VALUE;
	HANDLE h = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
			       nullptr, OPEN_EXISTING, 0, nullptr);
	if (h == INVALID_HANDLE_VALUE)
		return h;
	DCB dcb = {};
	dcb.DCBlength = sizeof(dcb);
	if (GetCommState(h, &dcb)) {
		dcb.BaudRate = CBR_115200; // nominal; USB-CDC ignores the rate
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;
		SetCommState(h, &dcb);
	}
	COMMTIMEOUTS to = {};
	to.ReadIntervalTimeout = 50;
	to.ReadTotalTimeoutConstant = 250;
	to.WriteTotalTimeoutConstant = 250;
	SetCommTimeouts(h, &to);
	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
	return h;
}

static bool moverio_serial_command(HANDLE h, const char *cmd, char *reply,
				   size_t reply_len)
{
	char buf[64];
	const int n = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
	if (n <= 0)
		return false;
	DWORD written = 0;
	if (!WriteFile(h, buf, static_cast<DWORD>(n), &written, nullptr))
		return false;
	if (!reply || reply_len == 0)
		return true;
	DWORD got = 0;
	if (!ReadFile(h, reply, static_cast<DWORD>(reply_len - 1), &got,
		      nullptr))
		return false;
	reply[got] = '\0';
	return got > 0;
}

// Query commands reply with a number as text ("0".."20", "50" = auto for
// getbright; "0"/"1" for getautobright). Framing is undocumented and the
// device may echo the command line first, so scan for the first digit and
// retry the read once when only the echo arrived in the first chunk.
static int moverio_query_value(HANDLE h, const char *cmd, int max_valid)
{
	char reply[64] = "";
	if (!moverio_serial_command(h, cmd, reply, sizeof(reply)))
		return -1;
	const char *p = reply;
	while (*p && !std::isdigit(static_cast<unsigned char>(*p)))
		++p;
	char more[32] = "";
	if (!*p) {
		DWORD got = 0;
		if (!ReadFile(h, more, sizeof(more) - 1, &got, nullptr) || !got)
			return -1;
		more[got] = '\0';
		p = more;
		while (*p && !std::isdigit(static_cast<unsigned char>(*p)))
			++p;
		if (!*p)
			return -1;
	}
	const int v = std::atoi(p);
	return (v >= 0 && v <= max_valid) ? v : -1;
}

static int moverio_query_brightness(HANDLE h)
{
	return moverio_query_value(h, "getbright", 50);
}

static int moverio_query_autobright(HANDLE h)
{
	return moverio_query_value(h, "getautobright", 1);
}

static void run_sensor_api_session(device_manager *f, uint32_t &seen_epoch,
				   uint64_t &last_detect_ns)
{
	const model_id m = detected_hid_model(f);
	if (m == MODEL_UNKNOWN)
		return;
	const device_entry &entry =
		g_device_registry[static_cast<size_t>(m) - 1];
	wchar_t vidpid[32];
	swprintf(vidpid, 32, L"vid_%04x&pid_%04x", entry.vid, entry.pid);

	const HRESULT co_hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
	const bool co_init = SUCCEEDED(co_hr);

	ISensorManager *mgr = nullptr;
	ISensor *gyro = nullptr;
	ISensor *accel = nullptr;
	ISensor *mag = nullptr;
	if (SUCCEEDED(CoCreateInstance(CLSID_SensorManager, nullptr,
				       CLSCTX_INPROC_SERVER,
				       IID_PPV_ARGS(&mgr))) &&
	    mgr) {
		gyro = find_device_sensor(mgr, SENSOR_TYPE_GYROMETER_3D, vidpid);
		accel = find_device_sensor(mgr, SENSOR_TYPE_ACCELEROMETER_3D,
					   vidpid);
		mag = find_device_sensor(mgr, SENSOR_TYPE_COMPASS_3D, vidpid);
	}

	// A sleeping device (display mute by stillness/tap detection) leaves
	// the sensors enumerable but not READY; treat that as not-connected so
	// we retry instead of blocking in GetData.
	if (!gyro || !accel || !sensor_is_ready(gyro)) {
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] Sensor API connect failed "
			     "(gyro=%d accel=%d ready=%d)",
			     gyro != nullptr, accel != nullptr,
			     gyro ? sensor_is_ready(gyro) : 0);
		if (gyro)
			gyro->Release();
		if (accel)
			accel->Release();
		if (mag)
			mag->Release();
		if (mgr)
			mgr->Release();
		if (co_init)
			CoUninitialize();
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	sensor_request_min_interval(gyro);
	sensor_request_min_interval(accel);
	if (mag)
		sensor_request_min_interval(mag);

	// Require a live gyro sample before declaring the session connected.
	// A wedged/sleeping device leaves the sensors enumerable and READY but
	// silent; without this gate the session flaps connected/disconnected
	// every watchdog period and the dock controls flicker with it.
	const double deg_to_rad = PI / 180.0;
	uint64_t gyro_ft100 = 0;
	vec3d g_latest = {};
	bool live = false;
	const uint64_t probe_start = os_gettime_ns();
	while (!f->stop.load(std::memory_order_relaxed) &&
	       os_gettime_ns() - probe_start < 1500000000ULL) {
		if (!sensor_is_ready(gyro)) {
			std::this_thread::sleep_for(
				std::chrono::milliseconds(50));
			continue;
		}
		if (sensor_read_vec3(
			    gyro,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_X_DEGREES_PER_SECOND,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Y_DEGREES_PER_SECOND,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Z_DEGREES_PER_SECOND,
			    gyro_ft100, g_latest)) {
			live = true;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	if (!live) {
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] Sensor API found sensors but "
			     "no live samples (device asleep or sensor stack "
			     "wedged); retrying");
		gyro->Release();
		accel->Release();
		if (mag)
			mag->Release();
		mgr->Release();
		if (co_init)
			CoUninitialize();
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] Sensor API connected (mag=%d)",
		     mag != nullptr);

	// Display-control serial port for brightness. Optional: the IMU works
	// without it and the dock disables the brightness row when absent.
	HANDLE serial = open_moverio_serial(entry.vid, entry.pid);
	f->brightness_current.store(
		serial != INVALID_HANDLE_VALUE ? moverio_query_brightness(serial)
					       : -1,
		std::memory_order_relaxed);
	f->autobright_current.store(
		serial != INVALID_HANDLE_VALUE ? moverio_query_autobright(serial)
					       : -1,
		std::memory_order_relaxed);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] MOVERIO serial %s (brightness %d, "
		     "autobright %d)",
		     serial != INVALID_HANDLE_VALUE ? "opened" : "not available",
		     f->brightness_current.load(std::memory_order_relaxed),
		     f->autobright_current.load(std::memory_order_relaxed));
	if (serial != INVALID_HANDLE_VALUE &&
	    f->debug_log.load(std::memory_order_relaxed) &&
	    f->brightness_current.load(std::memory_order_relaxed) < 0) {
		// Reply framing is undocumented; dump one raw reply to make
		// parser failures diagnosable from the log.
		char raw[64] = "";
		moverio_serial_command(serial, "getbright", raw, sizeof(raw));
		for (char *p = raw; *p; ++p)
			if (!std::isprint(static_cast<unsigned char>(*p)))
				*p = '.';
		blog(LOG_INFO, "[obs-nyan-real-3dof] getbright raw reply: '%s'",
		     raw);
	}

	uint64_t accel_ft100 = 0;
	uint64_t mag_ft100 = 0;
	vec3d a_latest = {};
	bool have_accel = false;
	uint32_t seq = 0;
	uint64_t last_rx_ns = os_gettime_ns();
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		refresh_detected_model(f, last_detect_ns);
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::sensor_api)
			break;
		const int bright_req = f->brightness_request.exchange(
			-1, std::memory_order_relaxed);
		if (bright_req >= 0 && serial != INVALID_HANDLE_VALUE) {
			char cmd[32];
			char reply[32] = "";
			snprintf(cmd, sizeof(cmd), "setbright %d", bright_req);
			moverio_serial_command(serial, cmd, reply, sizeof(reply));
			// Re-read instead of trusting the reply framing.
			const int now_bright = moverio_query_brightness(serial);
			f->brightness_current.store(
				now_bright >= 0 ? now_bright : bright_req,
				std::memory_order_relaxed);
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_INFO,
				     "[obs-nyan-real-3dof] setbright %d -> "
				     "reply '%s', getbright %d",
				     bright_req, reply, now_bright);
		}

		const int auto_req = f->autobright_request.exchange(
			-1, std::memory_order_relaxed);
		if (auto_req >= 0 && serial != INVALID_HANDLE_VALUE) {
			char cmd[32];
			char reply[32] = "";
			snprintf(cmd, sizeof(cmd), "enableautobright %d",
				 auto_req ? 1 : 0);
			moverio_serial_command(serial, cmd, reply, sizeof(reply));
			const int now_auto = moverio_query_autobright(serial);
			f->autobright_current.store(
				now_auto >= 0 ? now_auto : auto_req,
				std::memory_order_relaxed);
			// Leaving auto mode restores a manual level (and auto
			// mode reports 50); refresh the slider either way.
			const int now_bright = moverio_query_brightness(serial);
			if (now_bright >= 0)
				f->brightness_current.store(
					now_bright, std::memory_order_relaxed);
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_INFO,
				     "[obs-nyan-real-3dof] enableautobright %d "
				     "-> reply '%s', getautobright %d, "
				     "getbright %d",
				     auto_req, reply, now_auto, now_bright);
		}

		// The sensor objects go quiet rather than erroring when the
		// device misbehaves; reconnect like the other transports.
		if (os_gettime_ns() - last_rx_ns > 3000000000ULL) {
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] Sensor API stream "
				     "stalled (no samples for 3 s); reconnecting");
			break;
		}

		// While the device sleeps GetData would block indefinitely;
		// idle here and let the stall watchdog cycle the session.
		if (!sensor_is_ready(gyro)) {
			std::this_thread::sleep_for(
				std::chrono::milliseconds(50));
			continue;
		}

		if (sensor_read_vec3(accel, SENSOR_DATA_TYPE_ACCELERATION_X_G,
				     SENSOR_DATA_TYPE_ACCELERATION_Y_G,
				     SENSOR_DATA_TYPE_ACCELERATION_Z_G,
				     accel_ft100, a_latest))
			have_accel = true;

		if (sensor_read_vec3(
			    gyro,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_X_DEGREES_PER_SECOND,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Y_DEGREES_PER_SECOND,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Z_DEGREES_PER_SECOND,
			    gyro_ft100, g_latest) &&
		    have_accel) {
			last_rx_ns = os_gettime_ns();
			imu_sample imu;
			imu.gx = static_cast<float>(g_latest.x * deg_to_rad);
			imu.gy = static_cast<float>(g_latest.y * deg_to_rad);
			imu.gz = static_cast<float>(g_latest.z * deg_to_rad);
			// The accelerometer frame is the inverse of the
			// gyro/compass frame on MOVERIO; negate into one frame.
			imu.ax = static_cast<float>(-a_latest.x);
			imu.ay = static_cast<float>(-a_latest.y);
			imu.az = static_cast<float>(-a_latest.z);
			imu.ts_us = static_cast<uint32_t>(gyro_ft100 / 10);
			imu.seq = seq++;
			publish_sensor_samples(f, &imu, nullptr);
			maybe_log_sensor_rate(f, rate_log, "Sensor API");
		}

		vec3d mg = {};
		if (mag &&
		    sensor_read_vec3(
			    mag,
			    SENSOR_DATA_TYPE_MAGNETIC_FIELD_STRENGTH_X_MILLIGAUSS,
			    SENSOR_DATA_TYPE_MAGNETIC_FIELD_STRENGTH_Y_MILLIGAUSS,
			    SENSOR_DATA_TYPE_MAGNETIC_FIELD_STRENGTH_Z_MILLIGAUSS,
			    mag_ft100, mg)) {
			mag_sample ms;
			ms.mx = static_cast<float>(mg.x);
			ms.my = static_cast<float>(mg.y);
			ms.mz = static_cast<float>(mg.z);
			ms.temp_c = 0.0f;
			ms.ts_us = static_cast<uint32_t>(mag_ft100 / 10);
			ms.seq = seq;
			publish_sensor_samples(f, nullptr, &ms);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}

	if (serial != INVALID_HANDLE_VALUE)
		CloseHandle(serial);
	f->brightness_current.store(-1, std::memory_order_relaxed);
	f->autobright_current.store(-1, std::memory_order_relaxed);
	gyro->Release();
	accel->Release();
	if (mag)
		mag->Release();
	mgr->Release();
	if (co_init)
		CoUninitialize();
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] Sensor API disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

// Probe the HID interfaces of a rokid_hid model. The device streams without
// any start command, so accepting an interface only requires one decodable
// packet.
static HANDLE open_rokid_hid_stream(hid_interface_info &selected)
{
	for (const auto &info : enumerate_hid_interfaces()) {
		if (profile_for(info.model).transport != imu_transport::rokid_hid ||
		    is_consumer_control_hid(info))
			continue;
		HANDLE h = open_hid_path_rw(info.path);
		if (h == INVALID_HANDLE_VALUE)
			continue;

		bool ok = false;
		rokid_decode_state st;
		decoded_sensor_report decoded;
		std::vector<uint8_t> report;
		const uint64_t start = os_gettime_ns();
		while (!ok && os_gettime_ns() - start < 1200000000ULL) {
			if (!hid_read_report(h, info.input_report_len, report,
					     100))
				continue;
			if (decode_rokid_packet(report.data(), report.size(),
						st, decoded))
				ok = true;
		}
		if (ok) {
			selected = info;
			return h;
		}
		CloseHandle(h);
	}
	return INVALID_HANDLE_VALUE;
}

static void run_rokid_hid_session(device_manager *f, uint32_t &seen_epoch,
				  uint64_t &last_detect_ns)
{
	hid_interface_info info;
	HANDLE h = open_rokid_hid_stream(info);
	if (h == INVALID_HANDLE_VALUE) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] Rokid HID connect failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO, "[obs-nyan-real-3dof] Rokid HID connected");

	rokid_decode_state st;
	std::vector<uint8_t> report;
	uint32_t seq = 0;
	uint64_t last_rx_ns = os_gettime_ns();
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		refresh_detected_model(f, last_detect_ns);
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::rokid_hid)
			break;
		// There is no restart command to nudge a silent stream;
		// reconnect like the other transports.
		if (os_gettime_ns() - last_rx_ns > 3000000000ULL) {
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] Rokid HID stream "
				     "stalled (no samples for 3 s); reconnecting");
			break;
		}

		if (!hid_read_report(h, info.input_report_len, report, 250))
			continue;
		decoded_sensor_report decoded;
		if (!decode_rokid_packet(report.data(), report.size(), st,
					 decoded))
			continue;
		last_rx_ns = os_gettime_ns();
		decoded.imu.seq = seq;
		decoded.mag.seq = seq;
		seq++;
		publish_sensor_samples(f, decoded.has_imu ? &decoded.imu : nullptr,
				       decoded.has_mag ? &decoded.mag : nullptr);
		maybe_log_sensor_rate(f, rate_log, "Rokid HID");
	}

	CloseHandle(h);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] Rokid HID disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

// --- VITURE HID --------------------------------------------------------------
// VITURE glasses expose two vendor HID interfaces (usage page 0xFF00): MI_00
// streams IMU packets, MI_01 accepts MCU commands. All packets are 64 bytes:
// header FF FC = IMU data, FF FE = command; CRC-16-CCITT (poly 0x1021, init
// 0xFFFF) at offset 2 computed over everything from offset 4; payload length
// at offset 4 (LE); command id at 0x0E (LE); data at 0x12. Command 0x15 with
// one data byte starts (1) / stops (0) the fused-euler stream. Protocol facts
// from bfvogel/viture-webxr-extension and mgschwan/viture_virtual_display;
// the two sources disagree on the CRC byte order, so commands are sent in
// both orders (the device ignores packets with a bad CRC and the command is
// idempotent).

static uint16_t viture_crc16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < len; ++i) {
		crc = static_cast<uint16_t>(crc ^
					    (static_cast<uint16_t>(data[i]) << 8));
		for (int b = 0; b < 8; ++b)
			crc = (crc & 0x8000)
				      ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
				      : static_cast<uint16_t>(crc << 1);
	}
	return crc;
}

static std::vector<uint8_t> build_viture_command(uint16_t cmd_id,
						 const uint8_t *data,
						 size_t data_len, bool crc_be)
{
	std::vector<uint8_t> pkt(64, 0);
	pkt[0] = 0xFF;
	pkt[1] = VITURE_HDR_CMD;
	const uint16_t payload_len = static_cast<uint16_t>(0x0C + data_len);
	pkt[4] = static_cast<uint8_t>(payload_len & 0xFF);
	pkt[5] = static_cast<uint8_t>(payload_len >> 8);
	pkt[0x0E] = static_cast<uint8_t>(cmd_id & 0xFF);
	pkt[0x0F] = static_cast<uint8_t>(cmd_id >> 8);
	if (data_len)
		std::memcpy(&pkt[0x12], data, data_len);
	const uint16_t crc = viture_crc16(pkt.data() + 4,
					  static_cast<size_t>(payload_len) + 2);
	pkt[2] = static_cast<uint8_t>(crc_be ? (crc >> 8) : (crc & 0xFF));
	pkt[3] = static_cast<uint8_t>(crc_be ? (crc & 0xFF) : (crc >> 8));
	return pkt;
}

struct viture_iface {
	hid_interface_info info;
	HANDLE h = INVALID_HANDLE_VALUE;
};

// The MCU interface accepts the command and the IMU interface ignores it, so
// rather than depending on interface numbers the command goes to every
// interface, in both CRC byte orders.
static void viture_send_imu_enable(std::vector<viture_iface> &ifaces,
				   bool enable)
{
	const uint8_t data = enable ? 1 : 0;
	for (int crc_be = 1; crc_be >= 0; --crc_be) {
		const auto pkt = build_viture_command(VITURE_CMD_IMU, &data, 1,
						      crc_be != 0);
		for (auto &it : ifaces)
			hid_write_report(it.h, it.info.output_report_len, pkt,
					 250);
	}
}

// Strip the Windows report-id byte if present and decode a FF FC IMU packet
// into a body->world quaternion. The payload is three big-endian floats in
// degrees ordered roll, pitch, yaw (official SDK ordering); the composition
// and signs follow the reference WebXR extension's empirically confirmed
// mapping.
static bool decode_viture_packet(const uint8_t *data, size_t len, quatd &out)
{
	if (len >= 2 && data[0] == 0x00 && data[1] == 0xFF) {
		data++;
		len--;
	}
	if (len < 30 || data[0] != 0xFF || data[1] != VITURE_HDR_IMU)
		return false;
	const float roll = read_f32_be(data + 18);
	const float pitch = read_f32_be(data + 22);
	const float yaw = read_f32_be(data + 26);
	if (!std::isfinite(roll) || !std::isfinite(pitch) ||
	    !std::isfinite(yaw) || std::fabs(roll) > 720.0f ||
	    std::fabs(pitch) > 720.0f || std::fabs(yaw) > 720.0f)
		return false;
	const double d2r = PI / 180.0;
	out = quat_normalize(quat_multiply(
		quat_from_rot_z(-roll * d2r),
		quat_multiply(quat_from_yaw_y(yaw * d2r),
			      quat_from_rot_x(-pitch * d2r))));
	return true;
}

// Open every vendor interface of the detected VITURE device, request the IMU
// stream, and identify the streaming interface by probing for FF FC packets.
static bool open_viture_hid_stream(std::vector<viture_iface> &ifaces,
				   size_t &imu_index)
{
	for (const auto &info : enumerate_hid_interfaces()) {
		if (profile_for(info.model).transport !=
			    imu_transport::viture_hid ||
		    is_consumer_control_hid(info))
			continue;
		HANDLE h = open_hid_path_rw(info.path);
		if (h != INVALID_HANDLE_VALUE)
			ifaces.push_back({info, h});
	}
	if (ifaces.empty())
		return false;

	viture_send_imu_enable(ifaces, true);
	quatd q;
	std::vector<uint8_t> report;
	const uint64_t start = os_gettime_ns();
	while (os_gettime_ns() - start < 1500000000ULL) {
		for (size_t i = 0; i < ifaces.size(); ++i) {
			if (hid_read_report(ifaces[i].h,
					    ifaces[i].info.input_report_len,
					    report, 50) &&
			    decode_viture_packet(report.data(), report.size(),
						 q)) {
				imu_index = i;
				return true;
			}
		}
	}
	for (auto &it : ifaces)
		CloseHandle(it.h);
	ifaces.clear();
	return false;
}

static void run_viture_hid_session(device_manager *f, uint32_t &seen_epoch,
				   uint64_t &last_detect_ns)
{
	std::vector<viture_iface> ifaces;
	size_t imu_index = 0;
	if (!open_viture_hid_stream(ifaces, imu_index)) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] VITURE HID connect failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] VITURE HID connected "
		     "(interfaces=%u, imu=%u)",
		     static_cast<unsigned>(ifaces.size()),
		     static_cast<unsigned>(imu_index));

	std::vector<uint8_t> report;
	uint64_t last_rx_ns = os_gettime_ns();
	uint64_t last_nudge_ns = last_rx_ns;
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		refresh_detected_model(f, last_detect_ns);
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::viture_hid)
			break;
		const uint64_t now_ns = os_gettime_ns();
		if (now_ns - last_rx_ns > 3000000000ULL) {
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] VITURE HID stream "
				     "stalled (no samples for 3 s); reconnecting");
			break;
		}
		// Re-request the stream if it goes quiet (e.g. after the
		// glasses resume from standby).
		if (now_ns - last_rx_ns > 1200000000ULL &&
		    now_ns - last_nudge_ns > 1200000000ULL) {
			last_nudge_ns = now_ns;
			viture_send_imu_enable(ifaces, true);
		}

		if (!hid_read_report(ifaces[imu_index].h,
				     ifaces[imu_index].info.input_report_len,
				     report, 250))
			continue;
		quatd q;
		if (!decode_viture_packet(report.data(), report.size(), q))
			continue;
		last_rx_ns = os_gettime_ns();
		publish_external_pose(f, q, now_us32());
		maybe_log_sensor_rate(f, rate_log, "VITURE HID");
	}

	viture_send_imu_enable(ifaces, false);
	for (auto &it : ifaces)
		CloseHandle(it.h);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] VITURE HID disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

static void worker_fn(device_manager *f)
{
	uint32_t seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	uint64_t last_detect_ns = 0;
	while (!f->stop.load(std::memory_order_relaxed)) {
		refresh_detected_model(f, last_detect_ns);

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
			run_one_bridge_tcp_session(f, seen_epoch, last_detect_ns);
			break;
		case imu_transport::air_hid:
			run_air_hid_session(f, seen_epoch, last_detect_ns);
			break;
		case imu_transport::rayneo_hid:
			run_rayneo_hid_session(f, seen_epoch, last_detect_ns);
			break;
		case imu_transport::sensor_api:
			run_sensor_api_session(f, seen_epoch, last_detect_ns);
			break;
		case imu_transport::rokid_hid:
			run_rokid_hid_session(f, seen_epoch, last_detect_ns);
			break;
		case imu_transport::viture_hid:
			run_viture_hid_session(f, seen_epoch, last_detect_ns);
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

static quatd predict_pose(const pose_snapshot &p, float prediction_ms)
{
	quatd q = p.q;
	const double dt = clampd(prediction_ms, 0.0, 50.0) / 1000.0;
	const double wn =
		std::sqrt(p.omega.x * p.omega.x + p.omega.y * p.omega.y + p.omega.z * p.omega.z);
	if (p.calibrated && std::isfinite(wn) && wn > 1e-6 && dt > 0.0) {
		const double angle = wn * dt;
		const double h = 0.5 * angle;
		const double s = std::sin(h) / wn;
		const quatd dq = {std::cos(h), p.omega.x * s, p.omega.y * s,
				  p.omega.z * s};
		q = quat_normalize(quat_multiply(q, dq));
	}
	return q;
}

static const char *virtual_source_get_name(void *)
{
	return obs_module_text("source.name");
}

static bool is_virtual_source_id(const char *id)
{
	return id && strcmp(id, "nyan_real_3dof_virtual_screen") == 0;
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

static void manager_apply_settings(device_manager *f, obs_data_t *settings)
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
	f->mag_yaw.store(get_bool_setting(settings, "mag_yaw", false),
			 std::memory_order_relaxed);
	f->auto_projector.store(get_bool_setting(settings, "auto_projector", false),
				std::memory_order_relaxed);
	f->debug_log.store(get_bool_setting(settings, "debug_log", false),
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

static void manager_recenter(device_manager *f)
{
	std::lock_guard<std::mutex> lk(f->state_mutex);
	f->tracker.recenter();
	f->pose = f->tracker.snapshot();
	f->pose.connected = f->connected.load(std::memory_order_relaxed);
}

static void recenter_hotkey(void *data, obs_hotkey_id, obs_hotkey_t *, bool pressed)
{
	if (!pressed)
		return;
	manager_recenter(&g_device);
}

static void manager_recalibrate(device_manager *f)
{
	std::lock_guard<std::mutex> lk(f->state_mutex);
	f->tracker.restart_calibration();
	f->pose = f->tracker.snapshot();
	f->pose.connected = f->connected.load(std::memory_order_relaxed);
}

static void manager_apply_model_settings(device_manager *f)
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

static void manager_set_mag_yaw(device_manager *f, bool enabled)
{
	f->mag_yaw.store(enabled, std::memory_order_relaxed);
	std::lock_guard<std::mutex> lk(f->state_mutex);
	f->tracker.set_mag_yaw_enabled(enabled);
	f->pose = f->tracker.snapshot();
	f->pose.connected = f->connected.load(std::memory_order_relaxed);
}

static void manager_set_connect_enabled(device_manager *f, bool enabled)
{
	const bool prev = f->connect_enabled.exchange(enabled, std::memory_order_relaxed);
	if (prev != enabled)
		f->reconnect_epoch.fetch_add(1, std::memory_order_relaxed);
}

static void manager_set_network(device_manager *f, const std::string &ip, int port)
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

static void manager_reset_defaults(device_manager *f)
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
	f->mag_yaw.store(false, std::memory_order_relaxed);
	f->auto_projector.store(false, std::memory_order_relaxed);
	f->debug_log.store(false, std::memory_order_relaxed);

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

static void manager_save_load(obs_data_t *save_data, bool saving, void *)
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
		obs_data_set_bool(obj, "mag_yaw",
				  g_device.mag_yaw.load(std::memory_order_relaxed));
		obs_data_set_bool(obj, "auto_projector",
				  g_device.auto_projector.load(std::memory_order_relaxed));
		obs_data_set_bool(obj, "debug_log",
				  g_device.debug_log.load(std::memory_order_relaxed));
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

#ifdef NYAN_REAL_3DOF_WITH_QT_DOCK
// First nyan Real virtual screen source, used as the projector content.
static bool find_virtual_source_name(std::string &name_out)
{
	struct ctx_t {
		std::string name;
		bool found = false;
	} ctx;
	obs_enum_sources(
		[](void *param, obs_source_t *src) {
			auto *c = static_cast<ctx_t *>(param);
			if (!is_virtual_source_id(obs_source_get_id(src)) ||
			    obs_source_removed(src))
				return true;
			const char *n = obs_source_get_name(src);
			if (!n || !*n)
				return true;
			c->name = n;
			c->found = true;
			return false;
		},
		&ctx);
	if (ctx.found)
		name_out = ctx.name;
	return ctx.found;
}

// OBS resolves the fullscreen-projector monitor argument as an index into
// Qt's screen list. What QScreen::name() returns on Windows changed over
// time: before Qt 6.4 it was the GDI device name ("\\.\DISPLAY2"), from 6.4
// on it is the DISPLAYCONFIG friendly monitor name ("Air 2"). Try both, then
// fall back to the native geometry for duplicate or missing names.
static int projector_monitor_index(const nyan_real_glasses_display_info &display)
{
	const QList<QScreen *> screens = QGuiApplication::screens();

	const QString gdi = QString::fromStdString(display.gdi_device);
	for (int i = 0; i < screens.size(); ++i) {
		if (screens[i]->name() == gdi)
			return i;
	}

	const QString friendly = QString::fromStdString(display.friendly_name);
	int name_index = -1;
	int name_matches = 0;
	if (!friendly.isEmpty()) {
		for (int i = 0; i < screens.size(); ++i) {
			if (screens[i]->name() == friendly) {
				name_index = i;
				name_matches++;
			}
		}
	}
	if (name_matches == 1)
		return name_index;

	if (display.has_rect) {
		int size_index = -1;
		int size_matches = 0;
		for (int i = 0; i < screens.size(); ++i) {
			const QScreen *screen = screens[i];
			const qreal dpr = screen->devicePixelRatio();
			const QRect geo = screen->geometry();
			if (qRound(geo.width() * dpr) !=
				    static_cast<int>(display.width) ||
			    qRound(geo.height() * dpr) !=
				    static_cast<int>(display.height))
				continue;
			// Positions are exact at 100 % scaling; with mixed
			// per-monitor DPI Qt remaps origins, so accept a
			// small drift before falling back to a size-only
			// unique match.
			if (std::abs(qRound(geo.x() * dpr) - display.x) <= 2 &&
			    std::abs(qRound(geo.y() * dpr) - display.y) <= 2)
				return i;
			size_index = i;
			size_matches++;
		}
		if (size_matches == 1)
			return size_index;
	}

	return name_matches > 0 ? name_index : -1;
}

// Projector windows are OBS's own QWidget top-levels of class OBSProjector.
static QList<QWidget *> projectors_on_screen(QScreen *screen)
{
	QList<QWidget *> out;
	if (!screen)
		return out;
	const QList<QWidget *> widgets = QApplication::topLevelWidgets();
	for (QWidget *widget : widgets) {
		if (strcmp(widget->metaObject()->className(), "OBSProjector") !=
		    0)
			continue;
		if (widget->screen() != screen)
			continue;
		out.append(widget);
	}
	return out;
}

// Close existing projector windows on the target screen so the dock button
// and the auto-open never stack projectors. obs_frontend_open_projector opens
// a new window on every call, and OBS only deduplicates when the user enabled
// "Limit one fullscreen projector per screen".
static void close_projectors_on_screen(QScreen *screen)
{
	for (QWidget *widget : projectors_on_screen(screen))
		widget->close();
}

// Opens the virtual screen's fullscreen source projector on the glasses
// display. UI thread only.
static bool open_glasses_source_projector(bool log_failure)
{
	nyan_real_glasses_display_info display;
	if (!nyan_real_find_glasses_display(&display)) {
		if (log_failure)
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] no glasses display present (EDID match)");
		return false;
	}
	const int monitor = projector_monitor_index(display);
	if (monitor < 0) {
		if (log_failure) {
			std::string screen_names;
			for (QScreen *screen : QGuiApplication::screens()) {
				screen_names += '\'';
				screen_names += screen->name().toStdString();
				screen_names += "' ";
			}
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] glasses display %s ('%s') not matched; Qt screens: %s",
			     display.gdi_device.c_str(),
			     display.friendly_name.c_str(),
			     screen_names.c_str());
		}
		return false;
	}
	std::string source_name;
	if (!find_virtual_source_name(source_name)) {
		if (log_failure)
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] no virtual screen source exists; add one before opening the projector");
		return false;
	}
	close_projectors_on_screen(QGuiApplication::screens().value(monitor));
	obs_frontend_open_projector("Source", monitor, nullptr,
				    source_name.c_str());
	blog(LOG_INFO,
	     "[obs-nyan-real-3dof] opened source projector '%s' on %s (%s, monitor %d)",
	     source_name.c_str(), display.friendly_name.c_str(),
	     display.gdi_device.c_str(), monitor);
	return true;
}

class NoWheelSpinBox final : public QSpinBox {
public:
	using QSpinBox::QSpinBox;

protected:
	void wheelEvent(QWheelEvent *event) override { event->ignore(); }
};

class NoWheelDoubleSpinBox final : public QDoubleSpinBox {
public:
	using QDoubleSpinBox::QDoubleSpinBox;

protected:
	void wheelEvent(QWheelEvent *event) override { event->ignore(); }
};

class NyanRealDock final : public QScrollArea {
public:
	explicit NyanRealDock(QWidget *parent = nullptr) : QScrollArea(parent)
	{
		setWidgetResizable(true);
		setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
		setMinimumSize(240, 180);
		setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Ignored);

		auto *content = new QWidget(this);
		content->setMinimumWidth(220);
		auto *root = new QVBoxLayout(content);
		root->setContentsMargins(10, 10, 10, 10);
		root->setSpacing(8);

		auto *action_row_1 = new QHBoxLayout();
		auto *recenter = new QPushButton(obs_module_text("recenter"), content);
		auto *recalibrate = new QPushButton(obs_module_text("recalibrate"), content);
		recenter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
		recalibrate->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
		action_row_1->addWidget(recenter);
		action_row_1->addWidget(recalibrate);
		root->addLayout(action_row_1);


		auto *status_group = new QGroupBox(obs_module_text("dock.status"), content);
		auto *status_form = new QFormLayout(status_group);
		hid_label = new QLabel(status_group);
		glasses_display_label = new QLabel(status_group);
		transport_label = new QLabel(status_group);
		stream_label = new QLabel(status_group);
		pose_label = new QLabel(status_group);
		virtual_label = new QLabel(status_group);
		status_form->addRow(obs_module_text("dock.hid"), hid_label);
		status_form->addRow(obs_module_text("dock.glasses_display"),
				    glasses_display_label);
		status_form->addRow(obs_module_text("dock.transport"), transport_label);
		status_form->addRow(obs_module_text("dock.stream"), stream_label);
		status_form->addRow(obs_module_text("dock.pose"), pose_label);
		status_form->addRow(obs_module_text("dock.virtual_sources"),
				    virtual_label);
		root->addWidget(status_group);

		auto *device_group = new QGroupBox(obs_module_text("dock.device"), content);
		device_form = new QFormLayout(device_group);
		connect_box = new QCheckBox(obs_module_text("connect_enabled"), device_group);
		ip_edit = new QLineEdit(device_group);
		port_spin = new NoWheelSpinBox(device_group);
		port_spin->setRange(1, 65535);
		brightness_spin = new NoWheelDoubleSpinBox(device_group);
		brightness_spin->setRange(0.0, 20.0);
		brightness_spin->setDecimals(0);
		brightness_spin->setSingleStep(1.0);
		device_form->addRow(QString(), connect_box);
		device_form->addRow(obs_module_text("ip"), ip_edit);
		device_form->addRow(obs_module_text("port"), port_spin);
		brightness_row = make_double_slider(device_group, brightness_spin,
						    &brightness_slider,
						    BRIGHTNESS_SLIDER_SCALE);
		brightness_row->setToolTip(
			obs_module_text("brightness_tooltip"));
		device_form->addRow(obs_module_text("brightness"),
				    brightness_row);
		autobright_box = new QCheckBox(obs_module_text("autobright"),
					       device_group);
		autobright_box->setToolTip(
			obs_module_text("autobright_tooltip"));
		device_form->addRow(QString(), autobright_box);
		projector_button = new QPushButton(
			obs_module_text("dock.open_projector"), device_group);
		projector_button->setToolTip(
			obs_module_text("dock.open_projector_tooltip"));
		auto_projector_box = new QCheckBox(
			obs_module_text("dock.auto_projector"), device_group);
		auto_projector_box->setToolTip(
			obs_module_text("dock.auto_projector_tooltip"));
		device_form->addRow(QString(), projector_button);
		device_form->addRow(QString(), auto_projector_box);
		root->addWidget(device_group);

		auto *screen_group = new QGroupBox(obs_module_text("dock.screen"), content);
		auto *screen_form = new QFormLayout(screen_group);
		prediction_spin = new NoWheelDoubleSpinBox(screen_group);
		prediction_spin->setRange(0.0, 50.0);
		prediction_spin->setDecimals(0);
		prediction_spin->setSingleStep(1.0);
		fov_auto_box = new QCheckBox(obs_module_text("fov_auto"), screen_group);
		fov_spin = new NoWheelDoubleSpinBox(screen_group);
		fov_spin->setRange(20.0, 100.0);
		fov_spin->setDecimals(0);
		fov_spin->setSingleStep(1.0);
		distance_spin = new NoWheelDoubleSpinBox(screen_group);
		distance_spin->setRange(1.0, 10.0);
		distance_spin->setDecimals(1);
		distance_spin->setSingleStep(0.1);
		size_spin = new NoWheelDoubleSpinBox(screen_group);
		size_spin->setRange(0.25, 4.0);
		size_spin->setDecimals(2);
		size_spin->setSingleStep(0.05);
		curve_spin = new NoWheelDoubleSpinBox(screen_group);
		curve_spin->setRange(0.0, MAX_SCREEN_CURVE);
		curve_spin->setDecimals(2);
		curve_spin->setSingleStep(0.05);
		screen_label = new QLabel(screen_group);
		screen_form->addRow(obs_module_text("prediction_ms"),
				    make_double_slider(screen_group, prediction_spin,
						       &prediction_slider,
						       PREDICTION_SLIDER_SCALE));
		screen_form->addRow(QString(), fov_auto_box);
		screen_form->addRow(obs_module_text("fov_deg"),
				    make_double_slider(screen_group, fov_spin, &fov_slider,
						       FOV_SLIDER_SCALE));
		screen_form->addRow(obs_module_text("screen_distance_m"),
				    make_double_slider(screen_group, distance_spin,
						       &distance_slider,
						       DISTANCE_SLIDER_SCALE));
		screen_form->addRow(obs_module_text("screen_size_factor"),
				    make_double_slider(screen_group, size_spin, &size_slider,
						       SIZE_SLIDER_SCALE));
		screen_form->addRow(obs_module_text("screen_curve"),
				    make_double_slider(screen_group, curve_spin, &curve_slider,
						       CURVE_SLIDER_SCALE));
		screen_form->addRow(obs_module_text("dock.screen_result"), screen_label);
		root->addWidget(screen_group);

		auto *options_group = new QGroupBox(obs_module_text("dock.options"), content);
		auto *options_layout = new QVBoxLayout(options_group);
		mag_yaw_box = new QCheckBox(obs_module_text("mag_yaw"), options_group);
		debug_box = new QCheckBox(obs_module_text("debug_log"), options_group);
		// Resets every dock setting; lives at the bottom of the options
		// group, away from the everyday tracker buttons.
		auto *reset_defaults =
			new QPushButton(obs_module_text("reset_defaults"), options_group);
		options_layout->addWidget(mag_yaw_box);
		options_layout->addWidget(debug_box);
		options_layout->addWidget(reset_defaults);
		root->addWidget(options_group);
		root->addStretch();
		setWidget(content);

		QObject::connect(connect_box, &QCheckBox::toggled, this,
				 [](bool checked) { manager_set_connect_enabled(&g_device, checked); });
		QObject::connect(brightness_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.brightness_request.store(
						 static_cast<int>(
							 std::lround(value)),
						 std::memory_order_relaxed);
				 });
		QObject::connect(autobright_box, &QCheckBox::toggled, this,
				 [](bool checked) {
					 g_device.autobright_request.store(
						 checked ? 1 : 0,
						 std::memory_order_relaxed);
				 });
		QObject::connect(ip_edit, &QLineEdit::editingFinished, this, [this]() {
			manager_set_network(&g_device, ip_edit->text().trimmed().toStdString(),
					    port_spin->value());
		});
		QObject::connect(port_spin, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
				 this, [this](int value) {
					 manager_set_network(
						 &g_device,
						 ip_edit->text().trimmed().toStdString(), value);
				 });
		QObject::connect(recenter, &QPushButton::clicked, this,
				 []() { manager_recenter(&g_device); });
		QObject::connect(recalibrate, &QPushButton::clicked, this,
				 []() { manager_recalibrate(&g_device); });
		QObject::connect(projector_button, &QPushButton::clicked, this,
				 [this]() {
					 if (open_glasses_source_projector(true))
						 auto_projector_opened = true;
				 });
		QObject::connect(auto_projector_box, &QCheckBox::toggled, this,
				 [](bool checked) {
					 g_device.auto_projector.store(
						 checked,
						 std::memory_order_relaxed);
				 });
		QObject::connect(reset_defaults, &QPushButton::clicked, this, [this]() {
			manager_reset_defaults(&g_device);
			refresh();
		});
		QObject::connect(prediction_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.prediction_ms.store(static_cast<float>(value),
								      std::memory_order_relaxed);
				 });
		QObject::connect(fov_auto_box, &QCheckBox::toggled, this, [this](bool checked) {
			g_device.fov_auto.store(checked, std::memory_order_relaxed);
			set_double_enabled(fov_spin, fov_slider, !checked);
			if (checked)
				manager_apply_model_settings(&g_device);
		});
		QObject::connect(fov_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.fov_deg.store(static_cast<float>(value),
								std::memory_order_relaxed);
				 });
		QObject::connect(distance_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.screen_distance_m.store(
						 static_cast<float>(value),
						 std::memory_order_relaxed);
				 });
		QObject::connect(size_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.screen_size_factor.store(
						 static_cast<float>(value),
						 std::memory_order_relaxed);
				 });
		QObject::connect(curve_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.screen_curve.store(static_cast<float>(value),
								     std::memory_order_relaxed);
				 });
		QObject::connect(mag_yaw_box, &QCheckBox::toggled, this,
				 [](bool checked) { manager_set_mag_yaw(&g_device, checked); });
		QObject::connect(debug_box, &QCheckBox::toggled, this, [](bool checked) {
			g_device.debug_log.store(checked, std::memory_order_relaxed);
		});

		timer = new QTimer(this);
		QObject::connect(timer, &QTimer::timeout, this, [this]() { refresh(); });
		timer->start(500);
		refresh();
	}

	QSize sizeHint() const override
	{
		return QSize(320, 520);
	}

	QSize minimumSizeHint() const override
	{
		return QSize(220, 140);
	}

private:
	// One slider unit must equal the matching spin box singleStep
	// (scale = 1 / singleStep), so dragging the slider moves the value in
	// the same increments as the spin arrows. The spin box still accepts
	// finer values typed by hand.
	static constexpr int PREDICTION_SLIDER_SCALE = 1; // step 1 ms
	static constexpr int FOV_SLIDER_SCALE = 1;        // step 1 deg
	static constexpr int BRIGHTNESS_SLIDER_SCALE = 1; // step 1 level
	static constexpr int DISTANCE_SLIDER_SCALE = 10;  // step 0.1 m
	static constexpr int SIZE_SLIDER_SCALE = 20;      // step 0.05 x
	static constexpr int CURVE_SLIDER_SCALE = 20;     // step 0.05

	static int slider_value(double value, int scale)
	{
		return static_cast<int>(std::lround(value * static_cast<double>(scale)));
	}

	static QWidget *make_double_slider(QWidget *parent, QDoubleSpinBox *spin,
					   QSlider **slider_out, int scale)
	{
		auto *row = new QWidget(parent);
		auto *layout = new QHBoxLayout(row);
		layout->setContentsMargins(0, 0, 0, 0);
		layout->setSpacing(6);

		auto *slider = new QSlider(Qt::Horizontal, row);
		slider->setRange(slider_value(spin->minimum(), scale),
				 slider_value(spin->maximum(), scale));
		slider->setSingleStep(std::max(1, slider_value(spin->singleStep(), scale)));
		slider->setPageStep(std::max(slider->singleStep(),
					     slider_value(spin->singleStep() * 10.0, scale)));
		slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
		spin->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);

		layout->addWidget(slider, 1);
		layout->addWidget(spin);
		*slider_out = slider;

		QObject::connect(slider, &QSlider::valueChanged, row,
				 [spin, scale](int value) {
					 spin->setValue(static_cast<double>(value) /
							static_cast<double>(scale));
				 });
		QObject::connect(spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 row, [slider, scale](double value) {
					 const int next = slider_value(value, scale);
					 if (slider->value() == next)
						 return;
					 QSignalBlocker block(slider);
					 slider->setValue(next);
				 });
		return row;
	}

	static void set_spin(QDoubleSpinBox *spin, double value)
	{
		if (spin->hasFocus())
			return;
		QSignalBlocker block(spin);
		spin->setValue(value);
	}

	static void set_spin(QSpinBox *spin, int value)
	{
		if (spin->hasFocus())
			return;
		QSignalBlocker block(spin);
		spin->setValue(value);
	}

	static void set_double_control(QDoubleSpinBox *spin, QSlider *slider,
				       int scale, double value)
	{
		set_spin(spin, value);
		if (slider && !slider->hasFocus() && !slider->isSliderDown()) {
			QSignalBlocker block(slider);
			slider->setValue(slider_value(value, scale));
		}
	}

	static void set_double_enabled(QDoubleSpinBox *spin, QSlider *slider, bool enabled)
	{
		spin->setEnabled(enabled);
		if (slider)
			slider->setEnabled(enabled);
	}

	void refresh()
	{
		const model_id detected = detected_hid_model(&g_device);
		const bool connected = g_device.connected.load(std::memory_order_relaxed);
		const bool enabled = g_device.connect_enabled.load(std::memory_order_relaxed);
		const bool fov_auto = g_device.fov_auto.load(std::memory_order_relaxed);
		const double fov = clampd(g_device.fov_deg.load(std::memory_order_relaxed), 20.0,
					  100.0);
		const double distance =
			clampd(g_device.screen_distance_m.load(std::memory_order_relaxed), 1.0,
			       10.0);
		const double size_factor =
			clampd(g_device.screen_size_factor.load(std::memory_order_relaxed),
			       0.25, 4.0);
		const double screen_curve =
			clampd(g_device.screen_curve.load(std::memory_order_relaxed), 0.0,
			       MAX_SCREEN_CURVE);

		pose_snapshot p;
		{
			std::lock_guard<std::mutex> lk(g_device.state_mutex);
			p = g_device.pose;
		}

		hid_label->setText(detected == MODEL_UNKNOWN
					   ? obs_module_text("detected_device.none")
					   : QString::fromStdString(
						     profile_for(detected).name));
		// Transport-specific rows (currently the One-family TCP endpoint)
		// follow the detected device; nothing detected hides them all.
		const imu_transport transport = profile_for(detected).transport;
		transport_label->setText(
			obs_module_text(traits_for(transport).name_key));
		if (static_cast<int>(transport) != last_transport) {
			last_transport = static_cast<int>(transport);
			// No detected device means no IMU stream to enable; the
			// checkbox appears together with the device.
			device_form->setRowVisible(connect_box,
						   transport != imu_transport::none);
			const transport_traits tr = traits_for(transport);
			device_form->setRowVisible(ip_edit,
						   tr.uses_network_endpoint);
			device_form->setRowVisible(port_spin,
						   tr.uses_network_endpoint);
			device_form->setRowVisible(brightness_row,
						   tr.display_brightness);
			device_form->setRowVisible(autobright_box,
						   tr.display_brightness);
		}
		// Brightness is only adjustable while the session has the
		// serial command port open (-1 = unknown/unavailable) and the
		// device is not driving it from its ambient light sensor.
		const int brightness =
			g_device.brightness_current.load(std::memory_order_relaxed);
		const int autobright =
			g_device.autobright_current.load(std::memory_order_relaxed);
		autobright_box->setEnabled(autobright >= 0);
		if (g_device.autobright_request.load(std::memory_order_relaxed) <
		    0) {
			QSignalBlocker block(autobright_box);
			autobright_box->setChecked(autobright == 1);
		}
		set_double_enabled(brightness_spin, brightness_slider,
				   brightness >= 0 && autobright != 1);
		if (brightness >= 0 && brightness <= 20 &&
		    g_device.brightness_request.load(std::memory_order_relaxed) < 0)
			set_double_control(brightness_spin, brightness_slider,
					   BRIGHTNESS_SLIDER_SCALE, brightness);
		stream_label->setText(!enabled ? obs_module_text("dock.stream.disabled")
					       : (connected
							  ? obs_module_text("dock.stream.connected")
							  : obs_module_text("dock.stream.waiting")));
		const char *pose_status = obs_module_text("dock.pose.disconnected");
		if (!enabled) {
			pose_status = obs_module_text("dock.pose.disabled");
		} else if (connected && p.connected) {
			pose_status = p.calibrated
					      ? obs_module_text("dock.pose.calibrated")
					      : obs_module_text("dock.pose.calibrating");
		}
		pose_label->setText(pose_status);
		const int virtual_count =
			g_device.virtual_source_count.load(std::memory_order_relaxed);
		virtual_label->setText(QString::number(virtual_count));

		nyan_real_glasses_display_info glasses;
		const bool glasses_display_present =
			nyan_real_find_glasses_display(&glasses);
		const bool glasses_rect_valid =
			glasses_display_present && glasses.has_rect &&
			glasses.width > 0 && glasses.height > 0;
		g_glasses_display_width.store(glasses_rect_valid ? glasses.width
								 : 0,
					      std::memory_order_relaxed);
		g_glasses_display_height.store(glasses_rect_valid
						       ? glasses.height
						       : 0,
					       std::memory_order_relaxed);
		glasses_display_label->setText(
			glasses_display_present
				? QString::fromStdString(
					  glasses.friendly_name.empty()
						  ? glasses.gdi_device
						  : glasses.friendly_name)
				: QString(obs_module_text(
					  "dock.glasses_display.none")));
		// Disabled (not hidden) so the feature stays discoverable; the
		// "glasses display: not detected" status row explains why. The
		// auto-fullscreen checkbox below stays interactive because it is
		// a pre-arm setting for the next connection.
		projector_button->setEnabled(glasses_display_present);
		{
			QSignalBlocker block(auto_projector_box);
			auto_projector_box->setChecked(g_device.auto_projector.load(
				std::memory_order_relaxed));
		}
		if (!glasses_display_present) {
			// Windows moves a removed display's windows onto the
			// remaining monitors, which would leave the fullscreen
			// projector covering a desktop screen. Close the
			// projectors that were sitting on the glasses display
			// instead, and re-arm the auto-open for the next
			// connection.
			for (const QPointer<QWidget> &projector :
			     glasses_projectors) {
				if (projector)
					projector->close();
			}
			glasses_projectors.clear();
			auto_projector_opened = false;
		} else {
			if (g_device.auto_projector.load(
				    std::memory_order_relaxed) &&
			    !auto_projector_opened && detected != MODEL_UNKNOWN &&
			    virtual_count > 0) {
				if (open_glasses_source_projector(false))
					auto_projector_opened = true;
			}
			// Track every projector currently on the glasses
			// screen (manually opened ones included) so they can
			// be closed when the display disappears.
			glasses_projectors.clear();
			QScreen *glasses_screen = QGuiApplication::screens().value(
				projector_monitor_index(glasses));
			for (QWidget *projector :
			     projectors_on_screen(glasses_screen))
				glasses_projectors.append(projector);
		}

		{
			QSignalBlocker block(connect_box);
			connect_box->setChecked(enabled);
		}
		if (!ip_edit->hasFocus()) {
			std::lock_guard<std::mutex> lk(g_device.settings_mutex);
			QSignalBlocker block(ip_edit);
			ip_edit->setText(QString::fromStdString(g_device.ip));
			set_spin(port_spin, g_device.port);
		}
		{
			QSignalBlocker block(fov_auto_box);
			fov_auto_box->setChecked(fov_auto);
		}
		set_double_enabled(fov_spin, fov_slider, !fov_auto);
		set_double_control(prediction_spin, prediction_slider,
				   PREDICTION_SLIDER_SCALE,
				   g_device.prediction_ms.load(std::memory_order_relaxed));
		set_double_control(fov_spin, fov_slider, FOV_SLIDER_SCALE, fov);
		set_double_control(distance_spin, distance_slider, DISTANCE_SLIDER_SCALE,
				   distance);
		set_double_control(size_spin, size_slider, SIZE_SLIDER_SCALE, size_factor);
		set_double_control(curve_spin, curve_slider, CURVE_SLIDER_SCALE,
				   screen_curve);
		{
			QSignalBlocker block(mag_yaw_box);
			mag_yaw_box->setChecked(g_device.mag_yaw.load(std::memory_order_relaxed));
		}
		{
			QSignalBlocker block(debug_box);
			debug_box->setChecked(g_device.debug_log.load(std::memory_order_relaxed));
		}

		const double diag_m =
			2.0 * DEFAULT_SCREEN_DISTANCE_M * std::tan(fov * PI / 360.0) *
			size_factor;
		const double diag_in = diag_m / 0.0254;
		const double apparent_fov = 2.0 * std::atan(diag_m / (2.0 * distance)) *
					    180.0 / PI;
		screen_label->setText(QString::asprintf("%.1f in / %.1f deg", diag_in,
							apparent_fov));
	}

	QLabel *hid_label = nullptr;
	QLabel *glasses_display_label = nullptr;
	QLabel *transport_label = nullptr;
	QLabel *stream_label = nullptr;
	QLabel *pose_label = nullptr;
	QLabel *virtual_label = nullptr;
	QLabel *screen_label = nullptr;
	QCheckBox *connect_box = nullptr;
	QFormLayout *device_form = nullptr;
	QLineEdit *ip_edit = nullptr;
	QSpinBox *port_spin = nullptr;
	QDoubleSpinBox *brightness_spin = nullptr;
	QSlider *brightness_slider = nullptr;
	QWidget *brightness_row = nullptr;
	QCheckBox *autobright_box = nullptr;
	int last_transport = -1; // imu_transport value last applied to row visibility
	QDoubleSpinBox *prediction_spin = nullptr;
	QSlider *prediction_slider = nullptr;
	QCheckBox *fov_auto_box = nullptr;
	QDoubleSpinBox *fov_spin = nullptr;
	QSlider *fov_slider = nullptr;
	QDoubleSpinBox *distance_spin = nullptr;
	QSlider *distance_slider = nullptr;
	QDoubleSpinBox *size_spin = nullptr;
	QSlider *size_slider = nullptr;
	QDoubleSpinBox *curve_spin = nullptr;
	QSlider *curve_slider = nullptr;
	QCheckBox *mag_yaw_box = nullptr;
	QCheckBox *debug_box = nullptr;
	QPushButton *projector_button = nullptr;
	QCheckBox *auto_projector_box = nullptr;
	// Auto-open latch: one projector per glasses-display connection.
	bool auto_projector_opened = false;
	// Projectors seen on the glasses screen; closed on display removal.
	QList<QPointer<QWidget>> glasses_projectors;
	QTimer *timer = nullptr;
};

static void init_dock()
{
	auto *parent = reinterpret_cast<QWidget *>(obs_frontend_get_main_window());
	auto *widget = new NyanRealDock(parent);
	if (!obs_frontend_add_dock_by_id("nyan_real_3dof_dock",
					 obs_module_text("dock.title"), widget)) {
		delete widget;
		blog(LOG_ERROR,
		     "[obs-nyan-real-3dof] failed to register dock UI; check for duplicate plugin instances");
		return;
	}
	if (auto *dock = qobject_cast<QDockWidget *>(widget->parentWidget())) {
		dock->setAllowedAreas(Qt::AllDockWidgetAreas);
		dock->setFloating(false);
	} else {
		blog(LOG_WARNING,
		     "[obs-nyan-real-3dof] dock UI registered, but QDockWidget parent was not found");
	}
	blog(LOG_INFO,
	     "[obs-nyan-real-3dof] dock UI registered; enable it from the OBS Docks menu");
}

static void shutdown_dock()
{
	obs_frontend_remove_dock("nyan_real_3dof_dock");
}
#else
static void init_dock()
{
	blog(LOG_WARNING,
	     "[obs-nyan-real-3dof] Qt was not available at build time; dock UI disabled");
}

static void shutdown_dock() {}
#endif

static void bind_warp_effect(gs_effect_t *effect, gs_eparam_t **p_image,
			     gs_eparam_t **p_pose_q, gs_eparam_t **p_pose_valid,
			     gs_eparam_t **p_tan_half_fov,
			     gs_eparam_t **p_screen_distance_m,
			     gs_eparam_t **p_screen_half_size_m,
			     gs_eparam_t **p_screen_curve,
			     gs_eparam_t **p_debug_tint)
{
	if (!effect)
		return;
	if (p_image)
		*p_image = gs_effect_get_param_by_name(effect, "image");
	*p_pose_q = gs_effect_get_param_by_name(effect, "pose_q");
	*p_pose_valid = gs_effect_get_param_by_name(effect, "pose_valid");
	*p_tan_half_fov = gs_effect_get_param_by_name(effect, "tan_half_fov");
	*p_screen_distance_m =
		gs_effect_get_param_by_name(effect, "screen_distance_m");
	*p_screen_half_size_m =
		gs_effect_get_param_by_name(effect, "screen_half_size_m");
	*p_screen_curve = gs_effect_get_param_by_name(effect, "screen_curve");
	*p_debug_tint = gs_effect_get_param_by_name(effect, "debug_tint");
}

static gs_effect_t *create_warp_effect(gs_eparam_t **p_image,
				       gs_eparam_t **p_pose_q,
				       gs_eparam_t **p_pose_valid,
				       gs_eparam_t **p_tan_half_fov,
				       gs_eparam_t **p_screen_distance_m,
				       gs_eparam_t **p_screen_half_size_m,
				       gs_eparam_t **p_screen_curve,
				       gs_eparam_t **p_debug_tint)
{
	char *effect_path = obs_module_file("nyan-real-3dof.effect");
	gs_effect_t *effect = gs_effect_create_from_file(effect_path, nullptr);
	bfree(effect_path);
	bind_warp_effect(effect, p_image, p_pose_q, p_pose_valid, p_tan_half_fov,
			 p_screen_distance_m, p_screen_half_size_m, p_screen_curve,
			 p_debug_tint);
	return effect;
}

static void set_warp_effect_parameters(gs_eparam_t *p_pose_q,
				       gs_eparam_t *p_pose_valid,
				       gs_eparam_t *p_tan_half_fov,
				       gs_eparam_t *p_screen_distance_m,
				       gs_eparam_t *p_screen_half_size_m,
				       gs_eparam_t *p_screen_curve,
				       gs_eparam_t *p_debug_tint,
				       uint32_t view_w, uint32_t view_h,
				       uint32_t screen_w, uint32_t screen_h,
				       bool enable_pose)
{
	pose_snapshot p;
	{
		std::lock_guard<std::mutex> lk(g_device.state_mutex);
		p = g_device.pose;
	}
	const quatd q = predict_pose(
		p, g_device.prediction_ms.load(std::memory_order_relaxed));
	struct vec4 pose_q;
	pose_q.x = static_cast<float>(q.w);
	pose_q.y = static_cast<float>(q.x);
	pose_q.z = static_cast<float>(q.y);
	pose_q.w = static_cast<float>(q.z);
	gs_effect_set_vec4(p_pose_q, &pose_q);
	gs_effect_set_float(p_pose_valid,
			    (enable_pose && p.calibrated && p.connected) ? 1.0f
									 : 0.0f);

	// The global FOV value is the single source of truth for rendering. When
	// auto FOV is on, the resolved HID model writes its FOV into this value;
	// otherwise the dock's manual value is used.
	const float diagonal_fov_deg = static_cast<float>(
		clampd(g_device.fov_deg.load(std::memory_order_relaxed), 20.0, 100.0));
	const float view_aspect = view_h ? static_cast<float>(view_w) /
						 static_cast<float>(view_h)
					: 1.0f;
	const float screen_aspect = screen_h ? static_cast<float>(screen_w) /
						   static_cast<float>(screen_h)
					      : view_aspect;
	const float screen_height_factor =
		(view_h > 0 && screen_h > view_h)
			? static_cast<float>(screen_h) / static_cast<float>(view_h)
			: 1.0f;
	/* XREAL's public FOV is conventionally diagonal. Treat the UI value the
	 * same way and derive the viewer's horizontal/vertical tangents from the
	 * output aspect. The physical virtual screen keeps at least that viewer
	 * height, expands vertically when the referenced texture is taller than the
	 * output view, and uses the referenced texture's aspect. This lets multi-row
	 * display walls extend vertically instead of being squeezed into one view. */
	const float tan_diag =
		std::tan(diagonal_fov_deg * static_cast<float>(PI) / 360.0f);
	const float diag_scale = std::sqrt(view_aspect * view_aspect + 1.0f);
	const float tan_x = tan_diag * view_aspect / diag_scale;
	const float tan_y = tan_diag / diag_scale;
	const float screen_distance_m = static_cast<float>(
		clampd(g_device.screen_distance_m.load(std::memory_order_relaxed), 1.0,
		       10.0));
	const float screen_size_factor = static_cast<float>(
		clampd(g_device.screen_size_factor.load(std::memory_order_relaxed), 0.25,
		       4.0));
	const float screen_curve = static_cast<float>(
		clampd(g_device.screen_curve.load(std::memory_order_relaxed), 0.0,
		       MAX_SCREEN_CURVE));
	struct vec2 tan_half_fov;
	tan_half_fov.x = tan_x;
	tan_half_fov.y = tan_y;
	struct vec2 screen_half_size_m;
	screen_half_size_m.y = DEFAULT_SCREEN_DISTANCE_M * tan_y *
			       screen_size_factor * screen_height_factor;
	screen_half_size_m.x = screen_half_size_m.y * screen_aspect;
	gs_effect_set_vec2(p_tan_half_fov, &tan_half_fov);
	gs_effect_set_float(p_screen_distance_m, screen_distance_m);
	gs_effect_set_vec2(p_screen_half_size_m, &screen_half_size_m);
	gs_effect_set_float(p_screen_curve, screen_curve);
	gs_effect_set_float(p_debug_tint,
			    g_device.debug_log.load(std::memory_order_relaxed)
				    ? (p.connected ? 0.25f : 0.6f)
				    : 0.0f);
}

struct recursion_check_data {
	obs_source_t *needle = nullptr;
	bool found = false;
};

static void check_source_recursion(obs_source_t *parent, obs_source_t *child,
				   void *param)
{
	auto *d = static_cast<recursion_check_data *>(param);
	if (parent == d->needle || child == d->needle)
		d->found = true;
}

static bool virtual_target_allowed(const nyan_real_virtual_source *s,
				   obs_source_t *candidate)
{
	if (!candidate || obs_source_removed(candidate) || candidate == s->context)
		return false;
	if ((obs_source_get_output_flags(candidate) & OBS_SOURCE_VIDEO) == 0)
		return false;
	// Referencing another virtual screen is almost always an accidental
	// double-warp or a recursion path through a scene.
	if (is_virtual_source_id(obs_source_get_id(candidate)))
		return false;
	recursion_check_data check;
	check.needle = s->context;
	obs_source_enum_full_tree(candidate, check_source_recursion, &check);
	if (check.found)
		return false;
	return true;
}

static void virtual_source_remove_active_child(nyan_real_virtual_source *s)
{
	if (s->target && s->target_active_child) {
		obs_source_remove_active_child(s->context, s->target);
		s->target_active_child = false;
	}
}

static bool virtual_source_add_active_child(nyan_real_virtual_source *s)
{
	if (!s->target || s->target_active_child)
		return true;
	if (!obs_source_showing(s->context)) {
		s->target_recursion_blocked = false;
		return true;
	}

	if (!obs_source_add_active_child(s->context, s->target)) {
		s->target_recursion_blocked = true;
		blog(LOG_WARNING,
		     "[obs-nyan-real-3dof] virtual screen target rejected to avoid recursive rendering: %s",
		     obs_source_get_name(s->target));
		return false;
	}

	s->target_active_child = true;
	s->target_recursion_blocked = false;
	return true;
}

static void virtual_source_release_target(nyan_real_virtual_source *s)
{
	virtual_source_remove_active_child(s);
	if (s->target)
		obs_source_release(s->target);
	s->target = nullptr;
	s->target_recursion_blocked = false;
}

static void virtual_source_set_target(nyan_real_virtual_source *s,
				      const char *target_name,
				      bool log_failure = true)
{
	obs_source_t *next = nullptr;
	if (target_name && *target_name)
		next = obs_get_source_by_name(target_name);
	if (next && !virtual_target_allowed(s, next)) {
		obs_source_release(next);
		next = nullptr;
	}

	if (next == s->target) {
		if (next)
			obs_source_release(next);
		virtual_source_add_active_child(s);
		return;
	}

	virtual_source_release_target(s);
	s->target = next;
	if (s->target) {
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] virtual screen target set: '%s' (%ux%u)",
		     obs_source_get_name(s->target),
		     obs_source_get_width(s->target),
		     obs_source_get_height(s->target));
		virtual_source_add_active_child(s);
	} else if (log_failure && target_name && *target_name) {
		blog(LOG_WARNING,
		     "[obs-nyan-real-3dof] virtual screen target was not usable: '%s'",
		     target_name);
	}
}

static void virtual_source_update(void *data, obs_data_t *settings);

static void *virtual_source_create(obs_data_t *settings, obs_source_t *context)
{
	auto *s = new nyan_real_virtual_source();
	s->context = context;

	obs_enter_graphics();
	s->effect = create_warp_effect(&s->p_image, &s->p_pose_q, &s->p_pose_valid,
				       &s->p_tan_half_fov,
				       &s->p_screen_distance_m,
				       &s->p_screen_half_size_m,
				       &s->p_screen_curve,
				       &s->p_debug_tint);
	obs_leave_graphics();

	if (!s->effect) {
		blog(LOG_ERROR,
		     "[obs-nyan-real-3dof] nyan-real-3dof.effect missing -> virtual source disabled");
		delete s;
		return nullptr;
	}

	virtual_source_update(s, settings);
	g_device.virtual_source_count.fetch_add(1, std::memory_order_relaxed);
	blog(LOG_INFO, "[obs-nyan-real-3dof] virtual screen source created: %s",
	     BUILD_INFO);
	return s;
}

static void virtual_source_update(void *data, obs_data_t *settings)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	if (!settings) {
		s->target_name.clear();
		virtual_source_set_target(s, "");
		return;
	}
	const char *target_name = obs_data_get_string(settings, "target");
	s->target_name = target_name ? target_name : "";
	virtual_source_set_target(s, s->target_name.c_str());
	s->target_retry_timer_s = 0.0f;
}

static void virtual_source_destroy(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	virtual_source_release_target(s);
	obs_enter_graphics();
	if (s->texrender)
		gs_texrender_destroy(s->texrender);
	if (s->effect)
		gs_effect_destroy(s->effect);
	obs_leave_graphics();
	g_device.virtual_source_count.fetch_sub(1, std::memory_order_relaxed);
	delete s;
}

static uint32_t virtual_source_get_width(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	return s->output_width;
}

static uint32_t virtual_source_get_height(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	return s->output_height;
}

static void virtual_source_defaults(obs_data_t *settings)
{
	obs_data_set_default_string(settings, "target", "");
}

struct source_list_data {
	obs_property_t *list = nullptr;
	obs_source_t *self = nullptr;
	std::vector<std::string> names;
};

static bool add_source_to_property_list(void *data, obs_source_t *source)
{
	auto *d = static_cast<source_list_data *>(data);
	if (!source || source == d->self || obs_source_removed(source))
		return true;
	if ((obs_source_get_output_flags(source) & OBS_SOURCE_VIDEO) == 0)
		return true;
	if (is_virtual_source_id(obs_source_get_id(source)))
		return true;

	const char *name = obs_source_get_name(source);
	if (!name || !*name)
		return true;
	if (std::find(d->names.begin(), d->names.end(), name) != d->names.end())
		return true;
	d->names.emplace_back(name);
	return true;
}

static obs_properties_t *virtual_source_properties(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	obs_properties_t *props = obs_properties_create();
	obs_properties_add_text(props, "build_info", BUILD_INFO, OBS_TEXT_INFO);
	obs_properties_add_text(props, "source_global_notice",
				obs_module_text("source_global_notice"),
				OBS_TEXT_INFO);

	std::string render_notice =
		obs_module_text("source.render_resolution_notice");
	if (s) {
		render_notice += "\n";
		render_notice += std::to_string(s->output_width);
		render_notice += " x ";
		render_notice += std::to_string(s->output_height);
		render_notice += " px";
	}
	obs_properties_add_text(props, "source_render_resolution_notice",
				render_notice.c_str(), OBS_TEXT_INFO);
	obs_property_t *target = obs_properties_add_list(
		props, "target", obs_module_text("source.target"),
		OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);
	obs_property_list_add_string(target, obs_module_text("source.target.none"),
				     "");

	source_list_data list_data;
	list_data.list = target;
	list_data.self = s ? s->context : nullptr;
	obs_enum_scenes(add_source_to_property_list, &list_data);
	obs_enum_sources(add_source_to_property_list, &list_data);
	std::sort(list_data.names.begin(), list_data.names.end());
	for (const auto &name : list_data.names)
		obs_property_list_add_string(target, name.c_str(), name.c_str());

	std::string target_summary = obs_module_text("source.target_summary_none");
	if (s && s->target && !obs_source_removed(s->target)) {
		target_summary = obs_module_text("source.target_summary_prefix");
		target_summary += obs_source_get_name(s->target);
		target_summary += " (";
		target_summary += std::to_string(obs_source_get_width(s->target));
		target_summary += " x ";
		target_summary += std::to_string(obs_source_get_height(s->target));
		target_summary += " px)";
	}
	obs_properties_add_text(props, "source_target_summary",
				target_summary.c_str(), OBS_TEXT_INFO);
	return props;
}

static bool virtual_source_capture_target(nyan_real_virtual_source *s, uint32_t w,
					  uint32_t h,
					  enum gs_color_space space)
{
	const enum gs_color_format format = gs_get_format_from_space(space);
	if (s->texrender && gs_texrender_get_format(s->texrender) != format) {
		gs_texrender_destroy(s->texrender);
		s->texrender = nullptr;
	}
	if (!s->texrender)
		s->texrender = gs_texrender_create(format, GS_ZS_NONE);
	if (!s->texrender)
		return false;

	gs_texrender_reset(s->texrender);
	if (!gs_texrender_begin_with_color_space(s->texrender, w, h, space))
		return false;

	struct vec4 clear_color;
	vec4_zero(&clear_color);
	gs_clear(GS_CLEAR_COLOR, &clear_color, 0.0f, 0);
	gs_ortho(0.0f, static_cast<float>(w), 0.0f, static_cast<float>(h),
		 -100.0f, 100.0f);

	gs_blend_state_push();
	gs_blend_function_separate(GS_BLEND_SRCALPHA, GS_BLEND_INVSRCALPHA,
				   GS_BLEND_ONE, GS_BLEND_INVSRCALPHA);
	obs_source_video_render(s->target);
	gs_blend_state_pop();

	gs_texrender_end(s->texrender);
	return true;
}

static void virtual_source_draw_warp(nyan_real_virtual_source *s, gs_texture_t *tex,
				     uint32_t source_w, uint32_t source_h)
{
	set_warp_effect_parameters(s->p_pose_q, s->p_pose_valid, s->p_tan_half_fov,
				   s->p_screen_distance_m,
				   s->p_screen_half_size_m, s->p_screen_curve,
				   s->p_debug_tint,
				   s->output_width, s->output_height, source_w,
				   source_h, hid_device_ready(&g_device));

	const bool previous_srgb = gs_set_linear_srgb(true);
	const bool linear_srgb = gs_get_linear_srgb();
	const bool previous_fb = gs_framebuffer_srgb_enabled();
	gs_enable_framebuffer_srgb(linear_srgb);
	if (linear_srgb)
		gs_effect_set_texture_srgb(s->p_image, tex);
	else
		gs_effect_set_texture(s->p_image, tex);

	gs_technique_t *tech = gs_effect_get_technique(s->effect, "Draw");
	const size_t passes = gs_technique_begin(tech);
	for (size_t i = 0; i < passes; i++) {
		gs_technique_begin_pass(tech, i);
		gs_draw_sprite(tex, 0, s->output_width, s->output_height);
		gs_technique_end_pass(tech);
	}
	gs_technique_end(tech);
	gs_enable_framebuffer_srgb(previous_fb);
	gs_set_linear_srgb(previous_srgb);
}

static void virtual_source_render(void *data, gs_effect_t *)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	if (!s->effect || !s->p_image || !s->target || s->target_recursion_blocked ||
	    obs_source_removed(s->target))
		return;

	const uint32_t source_w = obs_source_get_width(s->target);
	const uint32_t source_h = obs_source_get_height(s->target);
	if (source_w == 0 || source_h == 0 || s->output_width == 0 ||
	    s->output_height == 0)
		return;

	const enum gs_color_space pref[] = {GS_CS_SRGB};
	const enum gs_color_space space =
		obs_source_get_color_space(s->target, 1, pref);
	if (!s->captured_this_frame) {
		if (!virtual_source_capture_target(s, source_w, source_h, space)) {
			const uint64_t now = os_gettime_ns();
			if (now - s->last_render_log_ns > 2000000000ULL) {
				s->last_render_log_ns = now;
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] virtual screen capture failed: target='%s' size=%ux%u",
				     obs_source_get_name(s->target), source_w,
				     source_h);
			}
			return;
		}
		s->captured_this_frame = true;
	}

	gs_texture_t *tex = gs_texrender_get_texture(s->texrender);
	if (!tex) {
		const uint64_t now = os_gettime_ns();
		if (now - s->last_render_log_ns > 2000000000ULL) {
			s->last_render_log_ns = now;
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] virtual screen texture was unavailable");
		}
		return;
	}

	gs_blend_state_push();
	gs_blend_function(GS_BLEND_ONE, GS_BLEND_INVSRCALPHA);
	virtual_source_draw_warp(s, tex, source_w, source_h);
	gs_blend_state_pop();
}

static void virtual_source_tick(void *data, float seconds)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	if (!s)
		return;
	s->captured_this_frame = false;

	// Render resolution is automatic: the glasses display's actual mode
	// when present, otherwise the HID-detected device's native resolution.
	uint32_t auto_w = g_glasses_display_width.load(std::memory_order_relaxed);
	uint32_t auto_h =
		g_glasses_display_height.load(std::memory_order_relaxed);
	if (!auto_w || !auto_h) {
		const model_profile &profile =
			profile_for(detected_hid_model(&g_device));
		auto_w = profile.display_width;
		auto_h = profile.display_height;
	}
	s->output_width = auto_w;
	s->output_height = auto_h;

	if (s->target_name.empty())
		return;

	if (s->target && obs_source_removed(s->target))
		virtual_source_release_target(s);

	if (s->target) {
		virtual_source_add_active_child(s);
		return;
	}

	s->target_retry_timer_s += seconds;
	if (s->target_retry_timer_s < VIRTUAL_TARGET_RETRY_INTERVAL_S)
		return;

	s->target_retry_timer_s = 0.0f;
	virtual_source_set_target(s, s->target_name.c_str(), false);
}

static void virtual_source_show(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	virtual_source_add_active_child(s);
	s->target_retry_timer_s = VIRTUAL_TARGET_RETRY_INTERVAL_S;
}

static void virtual_source_hide(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	virtual_source_remove_active_child(s);
	s->target_recursion_blocked = false;
}

static void virtual_source_enum_active(void *data,
				       obs_source_enum_proc_t enum_callback,
				       void *param)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	if (s->target && !s->target_recursion_blocked)
		enum_callback(s->context, s->target, param);
}

static bool virtual_source_audio_render(void *, uint64_t *,
					obs_source_audio_mix *, uint32_t, size_t,
					size_t)
{
	return false;
}

static enum gs_color_space virtual_source_get_color_space(void *, size_t,
						     const enum gs_color_space *)
{
	return GS_CS_SRGB;
}

// --- user-extensible device registry -------------------------------------
// devices.json next to the other plugin settings
// (<obs config>/plugin_config/obs-nyan-real-3dof/devices.json) can add device
// entries without rebuilding. See data/devices-example.json for the format.
// Broken entries are skipped one by one; the built-in table always remains.

// USB IDs are conventionally written in hex, so string values are parsed as
// hex with or without the 0x prefix. JSON numbers are taken as-is. For "pid",
// "*" / "any" / 0 match every PID of the VID.
static bool parse_usb_id(obs_data_t *item, const char *name, bool allow_any,
			 uint16_t &out)
{
	obs_data_item_t *it = obs_data_item_byname(item, name);
	if (!it)
		return false;
	long v = -1;
	const enum obs_data_type type = obs_data_item_gettype(it);
	if (type == OBS_DATA_STRING) {
		const char *s = obs_data_item_get_string(it);
		if (s && (strcmp(s, "*") == 0 || _stricmp(s, "any") == 0))
			v = 0;
		else if (s && *s)
			v = strtol(s, nullptr, 16);
	} else if (type == OBS_DATA_NUMBER) {
		v = static_cast<long>(obs_data_item_get_int(it));
	}
	obs_data_item_release(&it);
	if (v < 0 || v > 0xFFFF || (v == 0 && !allow_any))
		return false;
	out = static_cast<uint16_t>(v);
	return true;
}

static bool parse_transport(const char *s, imu_transport &out)
{
	if (!s || !*s)
		return false;
	if (_stricmp(s, "one_bridge_tcp") == 0 || _stricmp(s, "tcp") == 0) {
		out = imu_transport::one_bridge_tcp;
		return true;
	}
	if (_stricmp(s, "air_hid") == 0 || _stricmp(s, "hid") == 0) {
		out = imu_transport::air_hid;
		return true;
	}
	if (_stricmp(s, "rayneo_hid") == 0 || _stricmp(s, "rayneo") == 0) {
		out = imu_transport::rayneo_hid;
		return true;
	}
	if (_stricmp(s, "sensor_api") == 0 ||
	    _stricmp(s, "windows_sensor_api") == 0) {
		out = imu_transport::sensor_api;
		return true;
	}
	if (_stricmp(s, "rokid_hid") == 0 || _stricmp(s, "rokid") == 0) {
		out = imu_transport::rokid_hid;
		return true;
	}
	if (_stricmp(s, "viture_hid") == 0 || _stricmp(s, "viture") == 0) {
		out = imu_transport::viture_hid;
		return true;
	}
	return false;
}

// "mount" is either a preset name (standard / pro / air / rayneo) or a number
// of degrees (IMU mount rotation around X).
static bool parse_mount(obs_data_t *item, float &out_deg)
{
	obs_data_item_t *it = obs_data_item_byname(item, "mount");
	if (!it)
		return false;
	bool ok = false;
	const enum obs_data_type type = obs_data_item_gettype(it);
	if (type == OBS_DATA_STRING) {
		const char *s = obs_data_item_get_string(it);
		if (s && (_stricmp(s, "standard") == 0 ||
			  _stricmp(s, "one_standard") == 0)) {
			out_deg = MOUNT_X_DEG_ONE_STANDARD;
			ok = true;
		} else if (s && (_stricmp(s, "pro") == 0 ||
				 _stricmp(s, "one_pro") == 0)) {
			out_deg = MOUNT_X_DEG_ONE_PRO;
			ok = true;
		} else if (s && _stricmp(s, "air") == 0) {
			out_deg = MOUNT_X_DEG_AIR;
			ok = true;
		} else if (s && _stricmp(s, "rayneo") == 0) {
			out_deg = MOUNT_X_DEG_RAYNEO;
			ok = true;
		}
	} else if (type == OBS_DATA_NUMBER) {
		const double v = obs_data_item_get_double(it);
		if (std::isfinite(v) && v >= -360.0 && v <= 360.0) {
			out_deg = static_cast<float>(v);
			ok = true;
		}
	}
	obs_data_item_release(&it);
	return ok;
}

static void append_user_devices(std::vector<device_entry> &out,
				std::vector<nyan_real_glasses_display_id> &display_ids)
{
	char *path = obs_module_config_path("devices.json");
	if (!path)
		return;
	obs_data_t *root = obs_data_create_from_json_file(path);
	if (!root) {
		// Missing file is the normal case; nothing to report.
		bfree(path);
		return;
	}

	obs_data_array_t *arr = obs_data_get_array(root, "devices");
	const size_t count = arr ? obs_data_array_count(arr) : 0;
	size_t loaded = 0;
	for (size_t i = 0; i < count; ++i) {
		obs_data_t *item = obs_data_array_item(arr, i);
		if (!item)
			continue;
		device_entry e;
		const bool ok =
			parse_usb_id(item, "vid", false, e.vid) &&
			parse_usb_id(item, "pid", true, e.pid) &&
			parse_transport(obs_data_get_string(item, "transport"),
					e.profile.transport) &&
			parse_mount(item, e.profile.mount_x_deg);
		if (!ok) {
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] devices.json entry %zu ignored "
			     "(requires vid, pid, transport, mount)",
			     i);
			obs_data_release(item);
			continue;
		}
		const char *name = obs_data_get_string(item, "name");
		e.profile.name = (name && *name) ? name : "User device";
		double fov = obs_data_get_double(item, "fov_deg");
		if (fov <= 0.0)
			fov = 50.0;
		e.profile.fov_deg =
			static_cast<float>(fov < 20.0 ? 20.0
						      : (fov > 100.0 ? 100.0 : fov));
		const long long w = obs_data_get_int(item, "display_width");
		const long long h = obs_data_get_int(item, "display_height");
		e.profile.display_width = w > 0 ? static_cast<uint32_t>(w) : 1920;
		e.profile.display_height = h > 0 ? static_cast<uint32_t>(h) : 1080;
		const char *product = obs_data_get_string(item, "product_contains");
		if (product && *product) {
			wchar_t *wproduct = nullptr;
			os_utf8_to_wcs_ptr(product, 0, &wproduct);
			if (wproduct) {
				e.product_contains = wproduct;
				bfree(wproduct);
			}
		}
		// Optional EDID identity of the device's display panel, used by
		// the glasses auto-exclusion and the projector auto-open.
		nyan_real_glasses_display_id did;
		const char *edid_vendor = obs_data_get_string(item, "edid_vendor");
		if (edid_vendor && *edid_vendor)
			did.edid_vendor = nyan_real_pnp_vendor_word(edid_vendor);
		uint16_t edid_product = 0;
		if (parse_usb_id(item, "edid_product", true, edid_product))
			did.edid_product = edid_product;
		const char *edid_name =
			obs_data_get_string(item, "edid_name_contains");
		if (edid_name && *edid_name)
			did.name_contains = edid_name;
		if (did.edid_vendor || !did.name_contains.empty())
			display_ids.push_back(std::move(did));

		const char *transport_name = "one_bridge_tcp";
		if (e.profile.transport == imu_transport::sensor_api)
			transport_name = "sensor_api";
		else if (e.profile.transport == imu_transport::rokid_hid)
			transport_name = "rokid_hid";
		else if (e.profile.transport == imu_transport::air_hid)
			transport_name = "air_hid";
		else if (e.profile.transport == imu_transport::rayneo_hid)
			transport_name = "rayneo_hid";
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] user device: %s (%04X:%04X, %s)",
		     e.profile.name.c_str(), e.vid, e.pid, transport_name);
		out.push_back(std::move(e));
		loaded++;
		obs_data_release(item);
	}
	if (arr)
		obs_data_array_release(arr);
	obs_data_release(root);
	if (loaded > 0)
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] loaded %zu user device entries from %s",
		     loaded, path);
	bfree(path);
}

// Known glasses display panels by EDID. XREAL (formerly Nreal) uses the
// dedicated PNP id MRG for every generation (Air 0x3132, Air 2 0x3134,
// One Pro 0x4100, 1S 0x4102), so a vendor-only match is safe. RayNeo panels
// carry the parent company's TCL id, which also appears on ordinary TCL
// monitors, so they are qualified by product code or the "SmartGlasses" EDID
// monitor name.
static void append_builtin_glasses_display_ids(
	std::vector<nyan_real_glasses_display_id> &out)
{
	nyan_real_glasses_display_id xreal;
	xreal.edid_vendor = nyan_real_pnp_vendor_word("MRG");
	out.push_back(xreal);

	nyan_real_glasses_display_id rayneo;
	rayneo.edid_vendor = nyan_real_pnp_vendor_word("TCL");
	rayneo.edid_product = 0x03D4;
	out.push_back(rayneo);

	nyan_real_glasses_display_id rayneo_name;
	rayneo_name.edid_vendor = nyan_real_pnp_vendor_word("TCL");
	rayneo_name.name_contains = "SmartGlasses";
	out.push_back(rayneo_name);

	// EPSON MOVERIO BT-40 panel ("EPSON HMD"). SEC is Seiko Epson's PNP id
	// and is shared with their projectors, so qualify by product code.
	nyan_real_glasses_display_id moverio;
	moverio.edid_vendor = nyan_real_pnp_vendor_word("SEC");
	moverio.edid_product = 0xD004;
	out.push_back(moverio);

	// Rokid panels ("Rokid Max" = LBT 0x4753); match the family by monitor
	// name to cover other Rokid models on the same vendor id.
	nyan_real_glasses_display_id rokid;
	rokid.edid_vendor = nyan_real_pnp_vendor_word("LBT");
	rokid.name_contains = "Rokid";
	out.push_back(rokid);

	// VITURE panels report the generic CVT vendor id with "VITURE" as the
	// monitor name (VITURE One = CVT 0x3132), so qualify by name.
	nyan_real_glasses_display_id viture;
	viture.edid_vendor = nyan_real_pnp_vendor_word("CVT");
	viture.name_contains = "VITURE";
	out.push_back(viture);
}

// Build the device registry once, before the worker thread and the dock start
// reading it. User entries come first so they take precedence over built-ins.
static void init_device_registry()
{
	g_device_registry.clear();
	std::vector<nyan_real_glasses_display_id> display_ids;
	append_user_devices(g_device_registry, display_ids);
	append_builtin_devices(g_device_registry);
	append_builtin_glasses_display_ids(display_ids);
	nyan_real_set_glasses_display_ids(std::move(display_ids));
}

} // namespace

static obs_source_info nyan_real_3dof_virtual_info = {};
static obs_hotkey_id g_recenter_hotkey_id = OBS_INVALID_HOTKEY_ID;

bool obs_module_load(void)
{
	init_device_registry();

	nyan_real_3dof_virtual_info.id = "nyan_real_3dof_virtual_screen";
	nyan_real_3dof_virtual_info.type = OBS_SOURCE_TYPE_INPUT;
	nyan_real_3dof_virtual_info.output_flags =
		OBS_SOURCE_VIDEO | OBS_SOURCE_CUSTOM_DRAW | OBS_SOURCE_COMPOSITE |
		OBS_SOURCE_SRGB | OBS_SOURCE_DO_NOT_DUPLICATE;
	nyan_real_3dof_virtual_info.get_name = virtual_source_get_name;
	nyan_real_3dof_virtual_info.create = virtual_source_create;
	nyan_real_3dof_virtual_info.destroy = virtual_source_destroy;
	nyan_real_3dof_virtual_info.get_width = virtual_source_get_width;
	nyan_real_3dof_virtual_info.get_height = virtual_source_get_height;
	nyan_real_3dof_virtual_info.get_defaults = virtual_source_defaults;
	nyan_real_3dof_virtual_info.get_properties = virtual_source_properties;
	nyan_real_3dof_virtual_info.update = virtual_source_update;
	nyan_real_3dof_virtual_info.video_render = virtual_source_render;
	nyan_real_3dof_virtual_info.video_tick = virtual_source_tick;
	nyan_real_3dof_virtual_info.show = virtual_source_show;
	nyan_real_3dof_virtual_info.hide = virtual_source_hide;
	nyan_real_3dof_virtual_info.enum_active_sources = virtual_source_enum_active;
	nyan_real_3dof_virtual_info.enum_all_sources = virtual_source_enum_active;
	nyan_real_3dof_virtual_info.audio_render = virtual_source_audio_render;
	nyan_real_3dof_virtual_info.video_get_color_space =
		virtual_source_get_color_space;
	nyan_real_3dof_virtual_info.icon_type = OBS_ICON_TYPE_CUSTOM;
	obs_register_source(&nyan_real_3dof_virtual_info);

	register_nyan_real_display_wall_source();

	g_recenter_hotkey_id = obs_hotkey_register_frontend(
		"nyan_real_3dof.recenter", obs_module_text("hotkey.recenter"),
		recenter_hotkey, nullptr);

	obs_frontend_add_save_callback(manager_save_load, nullptr);
	init_dock();
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
	blog(LOG_INFO, "[obs-nyan-real-3dof] unloaded");
}
