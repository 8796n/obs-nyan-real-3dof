// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// OBS-independent wall placement math: turns selected/grouped monitors into a
// flat list of texture placements plus the total wall size. The host applies
// the placements to its capture children (OBS source items or the standalone
// app's own compositor); this layer touches no backend. Pairs with
// monitor_enum (which decides which monitors and how they are grouped).
#pragma once

#include <cstdint>
#include <vector>

#include "monitor_enum.h"

// Horizontal alignment of short rows within the wall (rows / auto layouts).
enum class row_align : int {
	left = 0,
	center = 1,
	right = 2,
};

// Placement of one monitor within the wall texture (top-left + size, px).
struct wall_placement {
	int x = 0;
	int y = 0;
	uint32_t width = 0;
	uint32_t height = 0;
};

// Computed wall geometry: the monitors in child order, their placements
// (parallel to monitors) and the total wall texture size.
struct wall_layout_result {
	std::vector<monitor_entry> monitors;
	std::vector<wall_placement> placements;
	uint32_t width = 0;
	uint32_t height = 0;
};

// Lays grouped rows left-to-right / top-to-bottom with the given gaps, outer
// padding and per-row horizontal alignment (rows / auto layout modes).
wall_layout_result compute_rows_layout(
	const std::vector<std::vector<monitor_entry>> &rows, int gap_x,
	int gap_y, int padding, row_align align);

// Mirrors the Windows desktop arrangement: each monitor keeps its relative
// position, with a gap inserted at every distinct column/row start.
wall_layout_result compute_windows_layout(
	const std::vector<monitor_entry> &monitors, int gap_x, int gap_y,
	int padding);
