// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>

#pragma once

#include <cstdint>
#include <string>
#include <vector>

void register_nyan_real_display_wall_source();

// EDID identity of an AR-glasses display panel. Every set field must match
// (AND); unset fields (0 / empty) are ignored. Entries with neither a vendor
// nor a name fragment never match. Monitor names alone are unreliable
// ("Air 2", "SmartGlasses"), so the EDID vendor/product ids are the primary
// key and the name fragment is a fallback qualifier.
struct nyan_real_glasses_display_id {
	uint16_t edid_vendor = 0;  // big-endian PNP word, e.g. "MRG" = 0x3647
	uint16_t edid_product = 0; // EDID product code, 0 = any
	std::string name_contains; // case-insensitive friendly-name fragment
};

// "MRG" -> 0x3647. Returns 0 for anything but three ASCII letters.
uint16_t nyan_real_pnp_vendor_word(const char *pnp);

// Installs the glasses-display identity list (user devices.json entries plus
// built-ins). Called once from obs_module_load before any source or dock
// exists; the list is immutable afterwards, so readers need no locking.
void nyan_real_set_glasses_display_ids(std::vector<nyan_real_glasses_display_id> ids);

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

// Desktop-to-wall-texture mapping of one wall monitor, published by the
// Display Wall whenever it lays out its children. The Audio Wall uses it to
// turn a window's desktop position into the horizontal texture coordinate
// the virtual screen actually renders, so audio bearings line up with the
// video without a manually tuned spread. With multiple Display Wall sources
// the most recent layout wins.
struct nyan_wall_monitor_map {
	long desk_left = 0; // physical desktop px
	long desk_right = 0;
	float u_left = 0.0f; // wall texture coordinate, 0..1
	float u_right = 0.0f;
};

// Copies up to max_count entries; returns the number copied (0 = no wall).
size_t nyan_real_get_wall_monitor_map(nyan_wall_monitor_map *out,
				      size_t max_count);
// Bumps on every published layout; cheap change detection for callers.
uint32_t nyan_real_wall_map_generation();
// Wall texture u (0..1) of the chosen center display's middle, published
// with the layout. -1 while the choice is "auto" or the chosen display is
// not part of the wall; the virtual screen then keeps the wall center
// forward. The render path turns this into the yaw it publishes as
// g_device.screen_yaw_offset_deg.
float nyan_real_wall_center_u();
