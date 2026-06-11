// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// The head-tracked virtual screen input source (backward warp shader).
#pragma once

#include <cstdint>
#include <cstring>

void register_nyan_real_virtual_source();

// Whether the virtual screen renders side-by-side for the given output size
// (the glasses display's mode when present). Shared with the dock so the
// IPD row's enabled state always matches what the renderer actually does.
bool sbs_output_active(uint32_t output_w, uint32_t output_h);

inline bool is_virtual_source_id(const char *id)
{
	return id && strcmp(id, "nyan_real_3dof_virtual_screen") == 0;
}
