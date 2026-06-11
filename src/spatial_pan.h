// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Shared head-tracked pan law for the spatial audio filter and the audio
// wall source: constant-power panning of a source anchored at a fixed
// bearing in the recentered virtual space, with a behind-the-head level cue
// and optional screen-distance attenuation. Header-only.
#pragma once

#include <cmath>
#include <mutex>

#include "device_manager.h"
#include "math_util.h"
#include "nyan_types.h"

constexpr int SPATIAL_MODE_POINT = 0;   // downmix to mono, then pan
constexpr int SPATIAL_MODE_BALANCE = 1; // keep stereo, attenuate one side

struct spatial_gains {
	float gl; // left output (pan + behind + distance)
	float gr; // right output
	float gx; // non-panned channels (behind + distance only)
};

// Head yaw in radians, positive = looking left (math_util convention).
// Without a valid pose the head is treated as facing forward, so sources
// stay at their configured bearings instead of jumping around.
inline double spatial_current_head_yaw()
{
	std::lock_guard<std::mutex> lk(g_device.state_mutex);
	if (g_device.pose.calibrated && g_device.pose.connected)
		return yaw_from_quat_heading(g_device.pose.q);
	return 0.0;
}

// bearing_deg is positive-right; yaw is positive-left, so a left head turn
// moves the source to the right of the gaze.
inline spatial_gains compute_spatial_gains(double bearing_deg, double yaw_rad,
					   int mode, bool distance_gain,
					   double *rel_out = nullptr)
{
	const double az = clampd(bearing_deg, -180.0, 180.0) * PI / 180.0;
	const double rel = wrap_angle(az + yaw_rad);
	if (rel_out)
		*rel_out = rel;
	const double arel = std::fabs(rel);
	const double half_pi = PI / 2.0;

	// Sources behind the head keep the fully-sided pan and lose a little
	// level (-2.5 dB directly behind) as a cheap "behind" cue.
	const double back =
		arel > half_pi
			? 1.0 - 0.25 * std::min(1.0, (arel - half_pi) / half_pi)
			: 1.0;

	double dist = 1.0;
	if (distance_gain) {
		const double d = clampd(
			g_device.screen_distance_m.load(std::memory_order_relaxed),
			MIN_SCREEN_DISTANCE_M, MAX_SCREEN_DISTANCE_M);
		// Inverse-distance law referenced to the 4 m default, capped
		// so the dock's distance slider cannot blow up the mix.
		dist = clampd(DEFAULT_SCREEN_DISTANCE_M / d, 0.25, 2.0);
	}

	// Constant-power pan: gl^2 + gr^2 = 1.
	const double p = std::sin(clampd(rel, -half_pi, half_pi));
	const double pan_arg = (p + 1.0) * PI / 4.0;
	double gl = std::cos(pan_arg);
	double gr = std::sin(pan_arg);
	if (mode == SPATIAL_MODE_BALANCE) {
		// Unity at center, attenuate only the far side.
		gl = std::min(1.0, gl * 1.41421356);
		gr = std::min(1.0, gr * 1.41421356);
	}
	return {static_cast<float>(gl * back * dist),
		static_cast<float>(gr * back * dist),
		static_cast<float>(back * dist)};
}
