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
