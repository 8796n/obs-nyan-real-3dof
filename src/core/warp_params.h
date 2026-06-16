// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Single source of truth for the 3DoF warp's per-frame parameters. Both the OBS
// source (virtual_source.cpp) and the standalone renderer compute the warp the
// same way by calling this - so they cannot drift (the gaze-dolly / center-
// offset / Audio-Wall publishes are all done here, once).
#pragma once

#include <cstdint>

#include "nyan_types.h" // quatd, vec3d

// Per-eye output geometry derived from the glasses framebuffer size.
struct eye_layout {
	bool sbs;           // side-by-side: two half-width eyes
	uint32_t eye_w;     // width of one eye's output region
	bool half_sbs;      // anamorphic half-SBS (glasses stretch each half)
	uint32_t eye_fov_w; // width the FOV/aspect math uses (un-squeezed)
};
eye_layout compute_eye_layout(uint32_t out_w, uint32_t out_h);

// Azimuth (radians, + = right) of a point offset off_m metres horizontally from
// the screen centre, seen on the flat (curve<=0) or cylindrical screen at
// dist_m. The single source of truth for "desktop x -> bearing": used by the
// warp's center-display offset and the Audio Wall's per-window bearing solver.
double screen_azimuth_rad(double off_m, double dist_m, double curve);

// Bearing (degrees, + = right) of a wall texture coordinate u (0..1 across the
// wall, may extend past either edge for off-wall positions) as rendered by the
// virtual screen. Reads the live geometry the warp publishes into g_device
// (half width, distance, curve, flat center-display shift, cylinder yaw) so the
// Audio Wall's per-window and per-tab bearings track the picture from one place;
// both backends and every input path call this after resolving their native
// coordinate to u (see wall_u_from_desktop_x).
double wall_u_to_bearing_deg(double u);

// Horizontal pixel disparity that converges a flat 2D overlay card at the
// virtual screen's depth in an SBS view: the left eye shifts the card right by
// this many pixels, the right eye left. 0 for degenerate inputs (mono callers
// should not invoke this; tan_half_fov_x / screen_dist_m / ipd_m <= 0). eye_w is
// one eye's output width in pixels; screen_dist_m and ipd_m are metres.
int overlay_convergence_px(uint32_t eye_w, double tan_half_fov_x,
			   double screen_dist_m, double ipd_m);

// Everything the warp shader needs for a frame, read from g_device + the view /
// screen sizes. Side effects (matching the OBS source): publishes
// g_device.screen_half_width_m and g_device.screen_yaw_offset_deg for the Audio
// Wall. The per-eye ray origin is viewer + sign*eye_right (sign -1 = left,
// +1 = right; mono uses viewer alone). pose_q is packed (w, x, y, z).
struct warp_params {
	float pose_q[4];
	float pose_valid;
	float tan_half_fov[2];
	float screen_distance_m;
	float screen_half_size_m[2];
	float screen_curve;
	float debug_tint;
	// Off-screen indicator: unit direction (+x right / +y up) toward the
	// virtual screen when it has left the view, and 0..1 strength (0 = visible
	// or pose invalid). The shader turns this into an edge glow.
	float offscreen_dir[2];
	float offscreen_intensity;
	float offscreen_band[2]; // glow thickness per axis (centered-coord units)
	// Flat-mirror aspect fit (used when pose_valid == 0): fraction of the
	// per-eye view the source fills per axis, preserving its aspect (the
	// shorter axis is letterboxed). 1,1 = fills exactly.
	float flat_fit[2];
	vec3d viewer;    // gaze-dolly viewer offset (remote-driven)
	vec3d eye_right; // rotate(pose, {IPD/2,0,0}); zero when mono
};

// eyes = compute_eye_layout(out_w, out_h); out_h = output height; screen_w/h =
// sampled (wall) texture size; enable_pose gates pose_valid; center_u in [0,1]
// rotates the chosen wall column to face forward (<0 disables the offset).
warp_params compute_warp_params(const eye_layout &eyes, uint32_t out_h,
				uint32_t screen_w, uint32_t screen_h,
				bool enable_pose, float center_u);
