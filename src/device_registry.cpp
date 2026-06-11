// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
#include "device_registry.h"

#include <obs-module.h>
#include <util/platform.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <string>

#include "display-wall-source.h"

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
		return {"transport.one_bridge_tcp", true, false, false};
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
	if (_stricmp(s, "nreal_hid") == 0 || _stricmp(s, "nreal") == 0) {
		out = imu_transport::nreal_hid;
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
		else if (e.profile.transport == imu_transport::viture_hid)
			transport_name = "viture_hid";
		else if (e.profile.transport == imu_transport::nreal_hid)
			transport_name = "nreal_hid";
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
