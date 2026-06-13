// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// EDID identity of AR-glasses display panels. This is device-registry
// knowledge (built from the built-in table plus user devices.json) and is
// OBS-independent: the device registry populates it, and consumers (the
// Display Wall's monitor filter, the dock's projector placement) match a
// physical monitor against it. Built once at startup and immutable after, so
// readers need no locking.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Every set field must match (AND); unset fields (0 / empty) are ignored.
// Entries with neither a vendor nor a name fragment never match. Monitor
// names alone are unreliable ("Air 2", "SmartGlasses"), so the EDID
// vendor/product ids are the primary key and the name fragment is a fallback.
struct nyan_real_glasses_display_id {
	uint16_t edid_vendor = 0;  // big-endian PNP word, e.g. "MRG" = 0x3647
	uint16_t edid_product = 0; // EDID product code, 0 = any
	std::string name_contains; // case-insensitive friendly-name fragment
};

// "MRG" -> 0x3647. Returns 0 for anything but three ASCII letters.
uint16_t nyan_real_pnp_vendor_word(const char *pnp);

// Installs the glasses-display identity list (user devices.json entries plus
// built-ins). Called once at startup before any consumer runs.
void nyan_real_set_glasses_display_ids(std::vector<nyan_real_glasses_display_id> ids);

// True if a monitor with this EDID vendor/product and friendly name matches a
// known glasses panel.
bool nyan_real_is_glasses_display(uint16_t edid_vendor, uint16_t edid_product,
				  const std::string &friendly_name);
