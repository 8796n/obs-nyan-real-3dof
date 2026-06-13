// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// OBS-independent monitor enumeration and identification: walks the Win32
// display APIs (EnumDisplayMonitors + QueryDisplayConfig) to list attached
// monitors with their geometry, Windows display number, friendly name and raw
// EDID ids, then filters/selects them by the dock's name/number tokens and
// groups them into rows. The Display Wall turns the result into capture
// children; the standalone app reuses the same layer. Pure data + Win32, no
// libobs.
#pragma once

#include <windows.h>

#include <cstdint>
#include <string>
#include <vector>

// One attached monitor as the wall sees it (native/physical desktop rect).
struct monitor_entry {
	int index = 0;
	int windows_number = 0;
	std::string id;
	std::string alt_id;
	std::string friendly_name;
	std::string device_name;
	std::string label;
	LONG x = 0;
	LONG y = 0;
	uint32_t width = 0;
	uint32_t height = 0;
	bool primary = false;
	uint16_t edid_vendor = 0;  // raw DISPLAYCONFIG edidManufactureId
	uint16_t edid_product = 0; // raw DISPLAYCONFIG edidProductCodeId
};

// One active display path from QueryDisplayConfig (friendly name + EDID ids,
// keyed by GDI device name). Used by the host's glasses-display lookup.
struct display_config_entry {
	std::string gdi_device;
	int windows_number = 0;
	std::string friendly_name;
	uint16_t edid_vendor = 0;  // raw DISPLAYCONFIG edidManufactureId
	uint16_t edid_product = 0; // raw DISPLAYCONFIG edidProductCodeId
};

// UTF-8 of a wide Win32 string ("" when null/empty).
std::string wide_to_utf8(const wchar_t *value);

// Active display paths, for friendly-name / EDID lookups by GDI device name.
std::vector<display_config_entry> get_active_display_config_entries();

// All attached monitors, sorted by Windows display number then enum order.
std::vector<monitor_entry> enumerate_monitors();

// Cheap topology snapshot (rects + device names) for periodic change detection;
// avoids the QueryDisplayConfig/EnumDisplayDevices cost of enumerate_monitors().
std::string monitor_topology_signature();

// Selects monitors by include-primary / exclude-glasses flags and the dock's
// include/exclude token filters (a Windows display number or a substring of the
// name/id).
std::vector<monitor_entry> filter_monitors(const std::vector<monitor_entry> &all,
					   bool include_primary,
					   bool exclude_glasses,
					   const std::string &filter,
					   const std::string &exclude_filter);

// Groups monitors into rows from a free-form layout string ("1,2/3"); a zero in
// any token switches the whole layout to zero-based numbering.
std::vector<std::vector<monitor_entry>>
build_rows_from_text(const std::vector<monitor_entry> &monitors,
		     const std::string &layout);

// Groups monitors into fixed-width rows (auto / rows layout modes).
std::vector<std::vector<monitor_entry>>
build_auto_rows(const std::vector<monitor_entry> &monitors, int columns);

// The glasses display the host should drive (EDID-matched), with its native
// virtual-desktop rect when resolvable.
struct nyan_real_glasses_display_info {
	std::string gdi_device;    // "\\.\DISPLAY4"
	std::string friendly_name; // EDID monitor name, e.g. "Air 2"
	int32_t x = 0;             // native (physical) virtual-desktop rect
	int32_t y = 0;
	uint32_t width = 0;
	uint32_t height = 0;
	bool has_rect = false;
};

// Finds the first active display whose EDID matches a known glasses entry.
// Returns false when no glasses display is present.
bool nyan_real_find_glasses_display(nyan_real_glasses_display_info *out);
