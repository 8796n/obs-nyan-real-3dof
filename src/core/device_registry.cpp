// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "device_registry.h"

#include <windows.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <string>

#include "glasses_display.h"
#include "nyan_json.h"
#include "nyan_log.h"
#include "nyan_paths.h"

constexpr uint16_t XREAL_VID = 0x3318;
constexpr uint16_t ASUS_VID = 0x0B05;
constexpr uint16_t RAYNEO_VID = 0x1BBB;
constexpr uint16_t EPSON_VID = 0x04B8;
constexpr uint16_t ROKID_VID = 0x04D2;
constexpr uint16_t VITURE_VID = 0x35CA;
// The Nreal Light is a USB hub of separate devices: the IMU streams from the
// OmniVision OV580 camera coprocessor, the STM32 MCU carries the command HID.
constexpr uint16_t NREAL_OV580_VID = 0x05A9;
constexpr uint16_t NREAL_OV580_PID = 0x0680;
constexpr uint16_t NREAL_MCU_VID = 0x0486;
constexpr uint16_t NREAL_MCU_PID = 0x573C;

std::vector<device_entry> g_device_registry;

const model_profile &profile_for(model_id m)
{
	static const model_profile unknown_profile = {imu_transport::none,
						      MOUNT_X_DEG_ONE_STANDARD,
						      50.0f, 1920, 1080, "Unknown"};
	if (m <= MODEL_UNKNOWN ||
	    static_cast<size_t>(m) > g_device_registry.size())
		return unknown_profile;
	return g_device_registry[static_cast<size_t>(m) - 1].profile;
}

// XREAL Air family display modes set over the MI_04 control interface
// (msgId 8). Values verified on Air 2 / Air 2 Pro hardware
// (MyGlasses2.0/analysis/12, 2026-04): 72/90Hz modes also lower the OLED
// duty cycle (low persistence). 120Hz needs an Air 2 panel; unsupported
// modes are rejected by the firmware and the GET refresh keeps the truth.
// Half-SBS (8) and SBS 90Hz (9) are omitted: the SET succeeds but the
// display stays blank (8 on Air 2 Pro, 9 on Air 2; both hardware-confirmed).
static const display_mode_option air_display_modes[] = {
	{1, "displaymode.mirror60"},  {5, "displaymode.mirror72"},
	{10, "displaymode.mirror90"}, {11, "displaymode.mirror120"},
	{3, "displaymode.sbs60"},     {4, "displaymode.sbs72"},
};

// Nreal Light display modes set over the MCU ASCII protocol (SET '3' with
// the ASCII digit of the value). The 540p half-SBS mode (2) is omitted.
// Mode 4 (SBS 72Hz) requires the ELLA hardware revision.
static const display_mode_option nreal_display_modes[] = {
	{1, "displaymode.mirror60"},
	{3, "displaymode.sbs60"},
	{4, "displaymode.sbs72"},
};

// VITURE resolution modes: the value is the protocol's ASCII state byte
// (SET cmd 0x08, '1' = 2D 1920x1080, '2' = 3D SBS 3840x1080), from
// disassembling the official Linux SDK 1.0.7. VITURE has no refresh-rate
// switching command, only this resolution toggle.
static const display_mode_option viture_display_modes[] = {
	{0x31, "displaymode.mode2d"},
	{0x32, "displaymode.mode3dsbs"},
};

transport_traits traits_for(imu_transport t)
{
	switch (t) {
	case imu_transport::one_bridge_tcp:
		return {"transport.one_bridge_tcp", true, false, false, nullptr,
			0, /*eye_camera=*/true};
	case imu_transport::air_hid:
		return {"transport.air_hid", false, true, false,
			air_display_modes, std::size(air_display_modes)};
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
		return {"transport.viture_hid", false, true, false,
			viture_display_modes, std::size(viture_display_modes)};
	case imu_transport::nreal_hid:
		return {"transport.nreal_hid", false, true, false,
			nreal_display_modes, std::size(nreal_display_modes)};
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
	// FOV per the published spec. Supports the SDK's setdisplaydistance
	// convergence command; its zero-shift convergence sits at 4.6 m per
	// the SDK appendix table, which is taken as the optical focal
	// distance too (the factory alignment of vergence and accommodation).
	model_profile bt40 = {imu_transport::sensor_api,
			      MOUNT_X_DEG_MOVERIO, 34.0f, 1920, 1080,
			      "EPSON MOVERIO BT-40"};
	bt40.optics_focus_m = 4.6f;
	bt40.display_distance = true;
	// EPSON MOVERIO BT-30C: same Sensor API IMU + serial command port
	// layout as the BT-40, on a 1280x720 panel with 23 deg diagonal FOV
	// per the published spec. USB product string "Moverio BT-30C HID-CDC"
	// (hardware-confirmed 2026-06).
	const model_profile bt30c = {imu_transport::sensor_api,
				     MOUNT_X_DEG_MOVERIO, 23.0f, 1280, 720,
				     "EPSON MOVERIO BT-30C"};
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
	// Nreal Light ("Nreal X" in mainland China): raw IMU from the OV580
	// coprocessor at 1000 Hz. 52 deg diagonal FOV per the published spec.
	// Registered under both the OV580 (the IMU stream the transport opens)
	// and the MCU HID so either interface identifies the device. 05A9 is
	// OmniVision's generic vendor id, so the OV580 row is qualified by its
	// product string.
	const model_profile nreal_light = {imu_transport::nreal_hid,
					   MOUNT_X_DEG_NREAL_LIGHT, 52.0f, 1920,
					   1080, "Nreal Light"};

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
	out.push_back({EPSON_VID, 0x0C0C, L"", bt30c});
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
	out.push_back({NREAL_OV580_VID, NREAL_OV580_PID, L"OV580", nreal_light});
	out.push_back({NREAL_MCU_VID, NREAL_MCU_PID, L"", nreal_light});
}

// --- user-extensible device registry -------------------------------------
// devices.json next to the other plugin settings
// (<obs config>/plugin_config/obs-nyan-real-3dof/devices.json) can add device
// entries without rebuilding. See data/devices-example.json for the format.
// Broken entries are skipped one by one; the built-in table always remains.

// --- JSON field helpers (no-throw; a missing or wrong-typed field yields the
// default, matching the old obs_data lookups). ------------------------------
static const nyan_json *json_find(const nyan_json &o, const char *key)
{
	if (!o.is_object())
		return nullptr;
	const auto it = o.find(key);
	return it == o.end() ? nullptr : &*it;
}

static std::string json_str(const nyan_json &o, const char *key)
{
	const nyan_json *v = json_find(o, key);
	return (v && v->is_string()) ? v->get<std::string>() : std::string();
}

static double json_num(const nyan_json &o, const char *key, double def)
{
	const nyan_json *v = json_find(o, key);
	return (v && v->is_number()) ? v->get<double>() : def;
}

static long long json_int(const nyan_json &o, const char *key, long long def)
{
	const nyan_json *v = json_find(o, key);
	if (!v)
		return def;
	if (v->is_number_integer())
		return v->get<long long>();
	if (v->is_number())
		return static_cast<long long>(v->get<double>());
	return def;
}

static bool json_bool(const nyan_json &o, const char *key, bool def)
{
	const nyan_json *v = json_find(o, key);
	return (v && v->is_boolean()) ? v->get<bool>() : def;
}

// UTF-8 -> wide for HID product-string matching (was os_utf8_to_wcs_ptr).
static std::wstring utf8_to_wide(const std::string &s)
{
	if (s.empty())
		return {};
	const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
	if (n <= 0)
		return {};
	std::wstring w(static_cast<size_t>(n), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, w.data(), n);
	if (!w.empty() && w.back() == L'\0')
		w.pop_back();
	return w;
}

// USB IDs are conventionally written in hex, so string values are parsed as
// hex with or without the 0x prefix. JSON numbers are taken as-is. For "pid",
// "*" / "any" / 0 match every PID of the VID.
static bool parse_usb_id(const nyan_json &item, const char *name, bool allow_any,
			 uint16_t &out)
{
	const nyan_json *v = json_find(item, name);
	if (!v)
		return false;
	long val = -1;
	if (v->is_string()) {
		const std::string s = v->get<std::string>();
		if (s == "*" || _stricmp(s.c_str(), "any") == 0)
			val = 0;
		else if (!s.empty())
			val = strtol(s.c_str(), nullptr, 16);
	} else if (v->is_number_integer()) {
		val = static_cast<long>(v->get<long long>());
	} else if (v->is_number()) {
		val = static_cast<long>(v->get<double>());
	}
	if (val < 0 || val > 0xFFFF || (val == 0 && !allow_any))
		return false;
	out = static_cast<uint16_t>(val);
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
	if (_stricmp(s, "nreal_hid") == 0 || _stricmp(s, "nreal") == 0) {
		out = imu_transport::nreal_hid;
		return true;
	}
	return false;
}

// "mount" is either a preset name (standard / pro / air / rayneo) or a number
// of degrees (IMU mount rotation around X).
static bool parse_mount(const nyan_json &item, float &out_deg)
{
	const nyan_json *v = json_find(item, "mount");
	if (!v)
		return false;
	if (v->is_string()) {
		const std::string s = v->get<std::string>();
		if (_stricmp(s.c_str(), "standard") == 0 ||
		    _stricmp(s.c_str(), "one_standard") == 0) {
			out_deg = MOUNT_X_DEG_ONE_STANDARD;
			return true;
		}
		if (_stricmp(s.c_str(), "pro") == 0 ||
		    _stricmp(s.c_str(), "one_pro") == 0) {
			out_deg = MOUNT_X_DEG_ONE_PRO;
			return true;
		}
		if (_stricmp(s.c_str(), "air") == 0) {
			out_deg = MOUNT_X_DEG_AIR;
			return true;
		}
		if (_stricmp(s.c_str(), "rayneo") == 0) {
			out_deg = MOUNT_X_DEG_RAYNEO;
			return true;
		}
		return false;
	}
	if (v->is_number()) {
		const double d = v->get<double>();
		if (std::isfinite(d) && d >= -360.0 && d <= 360.0) {
			out_deg = static_cast<float>(d);
			return true;
		}
	}
	return false;
}

static void append_user_devices(std::vector<device_entry> &out,
				std::vector<nyan_real_glasses_display_id> &display_ids)
{
	const std::string path = nyan_config_path("devices.json");
	if (path.empty())
		return;
	const nyan_json root = nyan_json_parse_file(path);
	// A missing or malformed file is the normal case; nothing to report.
	if (!root.is_object())
		return;
	const nyan_json *arr = json_find(root, "devices");
	if (!arr || !arr->is_array())
		return;

	size_t loaded = 0;
	size_t index = 0;
	for (const nyan_json &item : *arr) {
		const size_t i = index++;
		device_entry e;
		const bool ok =
			parse_usb_id(item, "vid", false, e.vid) &&
			parse_usb_id(item, "pid", true, e.pid) &&
			parse_transport(json_str(item, "transport").c_str(),
					e.profile.transport) &&
			parse_mount(item, e.profile.mount_x_deg);
		if (!ok) {
			nyan_log(NYAN_LOG_WARNING,
				 "[obs-nyan-real-3dof] devices.json entry %zu ignored "
				 "(requires vid, pid, transport, mount)",
				 i);
			continue;
		}
		const std::string name = json_str(item, "name");
		e.profile.name = !name.empty() ? name : "User device";
		double fov = json_num(item, "fov_deg", 0.0);
		if (fov <= 0.0)
			fov = 50.0;
		e.profile.fov_deg =
			static_cast<float>(fov < 20.0 ? 20.0
						      : (fov > 100.0 ? 100.0 : fov));
		const long long w = json_int(item, "display_width", 0);
		const long long h = json_int(item, "display_height", 0);
		e.profile.display_width = w > 0 ? static_cast<uint32_t>(w) : 1920;
		e.profile.display_height = h > 0 ? static_cast<uint32_t>(h) : 1080;
		// Optional optical virtual-image distance; 0 (absent) falls
		// back to DEFAULT_OPTICS_FOCUS_M through optics_focus().
		const double focus = json_num(item, "optics_focus_m", 0.0);
		if (std::isfinite(focus) && focus >= 0.5 && focus <= 20.0)
			e.profile.optics_focus_m = static_cast<float>(focus);
		// Optional MOVERIO setdisplaydistance (convergence) support;
		// only meaningful on sensor_api devices with the command port.
		e.profile.display_distance =
			json_bool(item, "display_distance", false);
		const std::string product = json_str(item, "product_contains");
		if (!product.empty())
			e.product_contains = utf8_to_wide(product);
		// Optional EDID identity of the device's display panel, used by
		// the glasses auto-exclusion and the projector auto-open.
		nyan_real_glasses_display_id did;
		const std::string edid_vendor = json_str(item, "edid_vendor");
		if (!edid_vendor.empty())
			did.edid_vendor =
				nyan_real_pnp_vendor_word(edid_vendor.c_str());
		uint16_t edid_product = 0;
		if (parse_usb_id(item, "edid_product", true, edid_product))
			did.edid_product = edid_product;
		const std::string edid_name = json_str(item, "edid_name_contains");
		if (!edid_name.empty())
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
		else if (e.profile.transport == imu_transport::viture_hid)
			transport_name = "viture_hid";
		else if (e.profile.transport == imu_transport::nreal_hid)
			transport_name = "nreal_hid";
		nyan_log(NYAN_LOG_INFO,
			 "[obs-nyan-real-3dof] user device: %s (%04X:%04X, %s)",
			 e.profile.name.c_str(), e.vid, e.pid, transport_name);
		out.push_back(std::move(e));
		loaded++;
	}
	if (loaded > 0)
		nyan_log(NYAN_LOG_INFO,
			 "[obs-nyan-real-3dof] loaded %zu user device entries from %s",
			 loaded, path.c_str());
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

	// EPSON MOVERIO panels ("EPSON HMD"). SEC is Seiko Epson's PNP id
	// and is shared with their projectors, so qualify by product code
	// (BT-40 = 0xD004, BT-30C = 0xD003; both hardware-confirmed).
	nyan_real_glasses_display_id moverio_bt40;
	moverio_bt40.edid_vendor = nyan_real_pnp_vendor_word("SEC");
	moverio_bt40.edid_product = 0xD004;
	out.push_back(moverio_bt40);

	nyan_real_glasses_display_id moverio_bt30c;
	moverio_bt30c.edid_vendor = nyan_real_pnp_vendor_word("SEC");
	moverio_bt30c.edid_product = 0xD003;
	out.push_back(moverio_bt30c);

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

	// The Nreal Light panel predates the MRG id: it reports NRL 0x3132
	// with "nreal light" as the monitor name (confirmed on hardware).
	// NRL is Nreal's own id, so a vendor-only match is safe.
	nyan_real_glasses_display_id nreal;
	nreal.edid_vendor = nyan_real_pnp_vendor_word("NRL");
	out.push_back(nreal);
}

// Build the device registry once, before the worker thread and the dock start
// reading it. User entries come first so they take precedence over built-ins.
void init_device_registry()
{
	g_device_registry.clear();
	std::vector<nyan_real_glasses_display_id> display_ids;
	append_user_devices(g_device_registry, display_ids);
	append_builtin_devices(g_device_registry);
	append_builtin_glasses_display_ids(display_ids);
	nyan_real_set_glasses_display_ids(std::move(display_ids));
}
