// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// OBS-independent wall placement math: turns selected/grouped monitors into a
// flat list of texture placements plus the total wall size. The host applies
// the placements to its capture children (OBS source items or the standalone
// app's own compositor); this layer touches no backend. Pairs with
// monitor_enum (which decides which monitors and how they are grouped).
#pragma once

#include <cstdint>
#include <string>
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
	// Holes left by excluded displays that sit *between* the wall monitors
	// (windows layout only): the host paints a "not part of the wall"
	// placeholder here so the gap reads as intentional instead of a black
	// void. Empty for rows / auto layouts (those pack monitors with no gaps).
	std::vector<wall_placement> blocked;
	uint32_t width = 0;
	uint32_t height = 0;
};

// A straight-alpha RGBA "keep out" placeholder for the holes excluded displays
// leave between wall monitors, so the gap reads as "not part of the wall". One
// baked image: a hazard zebra zone (equal-width 45-degree white stripes on black,
// per ISO 3864, framed like a road diversion zone / 導流帯) with the red
// prohibited sign painted opaquely on top of it (so the stripes never show
// through the sign), the whole thing given a uniform low alpha so it reads as a
// faint hint. Generated at the hole's exact pixel size and drawn 1:1 (alpha-
// blended), so the stripes keep their 45-degree angle and the sign stays round
// whatever the hole's aspect ratio.
struct wall_rgba_bitmap {
	int width = 0;
	int height = 0;
	std::vector<uint8_t> rgba; // width*height*4
};
wall_rgba_bitmap wall_keepout_fill(int width, int height);

// Lays grouped rows left-to-right / top-to-bottom with the given gaps, outer
// padding and per-row horizontal alignment (rows / auto layout modes).
wall_layout_result compute_rows_layout(
	const std::vector<std::vector<monitor_entry>> &rows, int gap_x,
	int gap_y, int padding, row_align align);

// Mirrors the Windows desktop arrangement: each monitor keeps its relative
// position, with a gap inserted at every distinct column/row start. `excluded`
// lists displays that are part of the desktop but not on the wall (the glasses,
// user-removed monitors); any whose center falls inside the laid-out monitors'
// bounding box becomes a `blocked` placeholder hole. Ones at the edge (which
// merely shrink the box) are skipped.
wall_layout_result compute_windows_layout(
	const std::vector<monitor_entry> &monitors, int gap_x, int gap_y,
	int padding, const std::vector<monitor_entry> &excluded = {});

// ---- desktop <-> wall-texture mapping ------------------------------------
// Maps one wall monitor's physical desktop x range to its horizontal range in
// the wall texture (0..1). The Audio Wall turns a window's desktop position
// into the texture coordinate the virtual screen actually renders, so audio
// bearings line up with the video; the center-display offset and the optional
// per-display border overlay read the same u ranges. OBS-independent so both
// backends derive the mapping from one place.
struct wall_monitor_map {
	long desk_left = 0;  // physical desktop px
	long desk_right = 0;
	float u_left = 0.0f; // wall texture coordinate 0..1
	float u_right = 0.0f;
};

// The horizontal map plus the chosen center display's middle u (-1 when the
// choice is "auto" or that display is not part of the wall; the virtual screen
// then keeps the wall center forward).
struct wall_audio_map {
	std::vector<wall_monitor_map> monitors;
	float center_u = -1.0f;
};

// Builds the desktop->wall-u map (and the center display's middle u) from a
// computed layout. monitors/placements are parallel; zero-width placements are
// skipped. center_display_id "" => center_u stays -1.
wall_audio_map compute_wall_audio_map(const wall_layout_result &layout,
				      const std::string &center_display_id);

// Desktop x -> wall texture u via nearest-monitor linear extrapolation:
// positions off the wall extend with the nearest monitor's scale (a screen to
// the right sounds from beyond the wall's right edge instead of merging with
// it), so the result may leave 0..1 and the caller bounds it. false when the
// map is empty.
bool wall_u_from_desktop_x(const std::vector<wall_monitor_map> &map, double x,
			   double *u_out);
