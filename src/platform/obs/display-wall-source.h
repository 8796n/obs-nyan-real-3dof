// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "glasses_display.h"
#include "wall_layout.h" // core wall_monitor_map (published below)

void register_nyan_real_display_wall_source();

// The Display Wall publishes its desktop->wall-texture mapping (core
// wall_monitor_map) whenever it lays out its children. The Audio Wall uses it
// to turn a window's desktop position into the horizontal texture coordinate
// the virtual screen actually renders, so audio bearings line up with the
// video without a manually tuned spread. With multiple Display Wall sources
// the most recent layout wins.

// Returns a copy of the current map (empty = no wall).
std::vector<wall_monitor_map> nyan_real_get_wall_monitor_map();
// Bumps on every published layout; cheap change detection for callers.
uint32_t nyan_real_wall_map_generation();
// Wall texture u (0..1) of the chosen center display's middle, published
// with the layout. -1 while the choice is "auto" or the chosen display is
// not part of the wall; the virtual screen then keeps the wall center
// forward. The render path turns this into the yaw it publishes as
// g_device.screen_yaw_offset_deg.
float nyan_real_wall_center_u();
