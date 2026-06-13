// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "glasses_display.h"

#include <algorithm>
#include <cctype>
#include <cstring>

namespace {

std::vector<nyan_real_glasses_display_id> g_glasses_display_ids;

std::string to_lower(std::string value)
{
	std::transform(value.begin(), value.end(), value.begin(),
		       [](unsigned char ch) {
			       return static_cast<char>(std::tolower(ch));
		       });
	return value;
}

// DISPLAYCONFIG reports edidManufactureId as the raw little-endian word of
// EDID bytes 8..9, while the PNP encoding reads them big-endian; accept both
// orders so the comparison never depends on that quirk.
bool vendor_word_matches(uint16_t edid_value, uint16_t expected_be)
{
	const uint16_t swapped = static_cast<uint16_t>(
		(edid_value >> 8) | static_cast<uint16_t>(edid_value << 8));
	return edid_value == expected_be || swapped == expected_be;
}

} // namespace

uint16_t nyan_real_pnp_vendor_word(const char *pnp)
{
	if (!pnp || std::strlen(pnp) != 3)
		return 0;
	uint16_t word = 0;
	for (int i = 0; i < 3; ++i) {
		const int c = std::toupper(static_cast<unsigned char>(pnp[i]));
		if (c < 'A' || c > 'Z')
			return 0;
		word = static_cast<uint16_t>((word << 5) |
					     static_cast<uint16_t>(c - 'A' + 1));
	}
	return word;
}

void nyan_real_set_glasses_display_ids(std::vector<nyan_real_glasses_display_id> ids)
{
	g_glasses_display_ids = std::move(ids);
}

bool nyan_real_is_glasses_display(uint16_t edid_vendor, uint16_t edid_product,
				  const std::string &friendly_name)
{
	for (const nyan_real_glasses_display_id &id : g_glasses_display_ids) {
		if (!id.edid_vendor && id.name_contains.empty())
			continue;
		if (id.edid_vendor &&
		    !vendor_word_matches(edid_vendor, id.edid_vendor))
			continue;
		if (id.edid_product && edid_product != id.edid_product)
			continue;
		if (!id.name_contains.empty() &&
		    to_lower(friendly_name).find(to_lower(id.name_contains)) ==
			    std::string::npos)
			continue;
		return true;
	}
	return false;
}
