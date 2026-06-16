// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "warp_params.h"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "device_manager.h" // g_device, sbs_output_active, pose_snapshot
#include "math_util.h"      // predict_pose, rotate_vector, clampd, quat_*, PI

double screen_azimuth_rad(double off_m, double dist_m, double curve)
{
	if (curve <= 0.0001)
		return std::atan2(off_m, dist_m);
	// Same cylinder as the warp shader: radius dist/curve, the screen centre
	// stays at dist_m, off_m is the arc length to the point.
	const double radius = dist_m / curve;
	const double theta = off_m / radius;
	return std::atan2(radius * std::sin(theta),
			  dist_m - radius * (1.0 - std::cos(theta)));
}

double wall_u_to_bearing_deg(double u)
{
	const double half_w =
		g_device.screen_half_width_m.load(std::memory_order_relaxed);
	const double dist = clampd(
		g_device.screen_distance_m.load(std::memory_order_relaxed),
		MIN_SCREEN_DISTANCE_M, MAX_SCREEN_DISTANCE_M);
	const double curve =
		clampd(g_device.screen_curve.load(std::memory_order_relaxed),
		       0.0, MAX_SCREEN_CURVE);
	const double center_off_m =
		g_device.screen_center_off_m.load(std::memory_order_relaxed);
	const double yaw_off =
		g_device.screen_yaw_offset_deg.load(std::memory_order_relaxed);
	// Up to half a wall width beyond each edge for off-wall displays; the
	// final bearing is the caller's to clamp if it needs to (the spatial
	// filter caps at +-90).
	const double off = (clampd(u, -0.5, 1.5) - 0.5) * 2.0 * half_w;
	return screen_azimuth_rad(off - center_off_m, dist, curve) * 180.0 / PI -
	       yaw_off;
}

int overlay_convergence_px(uint32_t eye_w, double tan_half_fov_x,
			   double screen_dist_m, double ipd_m)
{
	if (tan_half_fov_x <= 1e-4 || screen_dist_m <= 1e-3 || ipd_m <= 0.0)
		return 0;
	return static_cast<int>(static_cast<double>(eye_w) * ipd_m /
					(4.0 * screen_dist_m * tan_half_fov_x) +
				0.5);
}

eye_layout compute_eye_layout(uint32_t out_w, uint32_t out_h)
{
	eye_layout l;
	// Full SBS (double-wide, e.g. 3840x1080) maps each half 1:1, so the FOV
	// math uses the half width. Half-SBS (manual ON at a normal aspect, e.g.
	// 1920x1080) is anamorphic - the glasses stretch each half back to the full
	// panel width - so the FOV math must use the un-squeezed full width.
	l.sbs = sbs_output_active(out_w, out_h);
	l.eye_w = l.sbs ? out_w / 2 : out_w;
	l.half_sbs = l.sbs && out_w < out_h * 3;
	l.eye_fov_w = l.half_sbs ? out_w : l.eye_w;
	return l;
}

warp_params compute_warp_params(const eye_layout &eyes, uint32_t out_h,
				uint32_t screen_w, uint32_t screen_h,
				bool enable_pose, float center_u)
{
	const uint32_t view_w = eyes.eye_fov_w;
	const uint32_t view_h = out_h;

	pose_snapshot p;
	{
		std::lock_guard<std::mutex> lk(g_device.state_mutex);
		p = g_device.pose;
	}
	quatd q = predict_pose(
		p, g_device.prediction_ms.load(std::memory_order_relaxed));

	warp_params wp = {};
	// pose_follow off = head-locked flat mirror (the passthrough path).
	wp.pose_valid = (enable_pose && p.calibrated && p.connected &&
			 g_device.pose_follow.load(std::memory_order_relaxed))
				? 1.0f
				: 0.0f;

	// The global FOV value is the single source of truth: auto FOV writes the
	// resolved HID model's FOV here, otherwise the dock's manual value is used.
	// XREAL's public FOV is diagonal; derive the horizontal/vertical tangents
	// from the output aspect. The physical screen keeps at least that viewer
	// height, expands when the texture is taller than the view (multi-row
	// walls), and uses the texture's aspect.
	const float diagonal_fov_deg = static_cast<float>(clampd(
		g_device.fov_deg.load(std::memory_order_relaxed), 20.0, 100.0));
	const float view_aspect =
		view_h ? static_cast<float>(view_w) / static_cast<float>(view_h)
		       : 1.0f;
	const float screen_aspect =
		screen_h ? static_cast<float>(screen_w) /
				   static_cast<float>(screen_h)
			 : view_aspect;
	const float screen_height_factor =
		(view_h > 0 && screen_h > view_h)
			? static_cast<float>(screen_h) / static_cast<float>(view_h)
			: 1.0f;
	const float tan_diag =
		std::tan(diagonal_fov_deg * static_cast<float>(PI) / 360.0f);
	const float diag_scale = std::sqrt(view_aspect * view_aspect + 1.0f);
	wp.tan_half_fov[0] = tan_diag * view_aspect / diag_scale;
	wp.tan_half_fov[1] = tan_diag / diag_scale;
	wp.screen_distance_m = static_cast<float>(
		clampd(g_device.screen_distance_m.load(std::memory_order_relaxed),
		       MIN_SCREEN_DISTANCE_M, MAX_SCREEN_DISTANCE_M));
	const float screen_size_factor = static_cast<float>(
		clampd(g_device.screen_size_factor.load(std::memory_order_relaxed),
		       0.05, 4.0));
	wp.screen_curve = static_cast<float>(
		clampd(g_device.screen_curve.load(std::memory_order_relaxed), 0.0,
		       MAX_SCREEN_CURVE));
	// The size factor scales against the fixed unit distance (factor 1.0 fills
	// the FOV seen from that distance), independent of the current distance -
	// moving the distance slider keeps the physical size and changes apparent size.
	wp.screen_half_size_m[1] = static_cast<float>(SCREEN_SIZE_UNIT_DISTANCE_M) *
				   wp.tan_half_fov[1] * screen_size_factor *
				   screen_height_factor;
	wp.screen_half_size_m[0] = wp.screen_half_size_m[1] * screen_aspect;
	// Flat-mirror (pose-off) aspect fit: contain the source in the per-eye view
	// without stretching. The axis whose source/view ratio exceeds 1 fills the
	// view (fraction 1); the other axis is letterboxed to keep the aspect.
	wp.flat_fit[0] = std::min(1.0f, screen_aspect / view_aspect);
	wp.flat_fit[1] = std::min(1.0f, view_aspect / screen_aspect);
	// Published for the Audio Wall's geometric bearing computation.
	g_device.screen_half_width_m.store(wp.screen_half_size_m[0],
					   std::memory_order_relaxed);

	// Center-display offset (horizontal only): put the chosen wall column
	// straight ahead after a recenter. You cannot face a flat surface by
	// rotating, so the two screen shapes need different moves:
	//   - Cylinder: yaw the world to the chosen column (it stays equidistant /
	//     face-on at the cylinder centre).
	//   - Flat: slide the viewer sideways to stand in front of the column, so it
	//     is seen face-on instead of obliquely (a yaw would keystone it).
	// Both are published so the Audio Wall derives the same bearings.
	double yaw_off = 0.0;     // cylinder
	double center_off_m = 0.0; // flat (lateral viewer shift, metres)
	if (center_u >= 0.0f && wp.screen_half_size_m[0] > 1e-6f) {
		const double off = (clampd(center_u, 0.0, 1.0) - 0.5) * 2.0 *
				   wp.screen_half_size_m[0];
		if (wp.screen_curve <= 0.0001f)
			center_off_m = off;
		else
			yaw_off = screen_azimuth_rad(off, wp.screen_distance_m,
						     wp.screen_curve);
	}
	g_device.screen_yaw_offset_deg.store(
		static_cast<float>(yaw_off * 180.0 / PI), std::memory_order_relaxed);
	g_device.screen_center_off_m.store(static_cast<float>(center_off_m),
					   std::memory_order_relaxed);
	if (yaw_off != 0.0)
		q = quat_normalize(quat_multiply(quat_from_yaw_y(-yaw_off), q));

	wp.pose_q[0] = static_cast<float>(q.w);
	wp.pose_q[1] = static_cast<float>(q.x);
	wp.pose_q[2] = static_cast<float>(q.y);
	wp.pose_q[3] = static_cast<float>(q.z);
	wp.debug_tint = g_device.debug_log.load(std::memory_order_relaxed)
				? (p.connected ? 0.25f : 0.6f)
				: 0.0f;

	// Per-eye parallax: each eye renders from its own world position (the
	// head-frame +/-IPD/2 lateral offset rotated by the pose). Mono keeps the
	// single center. The gaze-dolly viewer offset shifts both eyes alike.
	const double half_ipd_m =
		eyes.sbs ? clampd(g_device.ipd_mm.load(std::memory_order_relaxed),
				  MIN_IPD_MM, MAX_IPD_MM) *
				   0.0005
			 : 0.0;
	wp.eye_right = rotate_vector(q, {half_ipd_m, 0.0, 0.0});
	// The flat center-display shift adds to the gaze-dolly viewer offset so the
	// eye sits in front of the chosen display (cylinder uses yaw, so 0 there).
	wp.viewer = {g_device.viewer_offset_x.load(std::memory_order_relaxed) +
			     center_off_m,
		     g_device.viewer_offset_y.load(std::memory_order_relaxed),
		     g_device.viewer_offset_z.load(std::memory_order_relaxed)};

	// Off-screen indicator: the virtual screen is a whole Display Wall (possibly
	// several monitors wide / multi-row), so a single-point or tangent-rect test
	// breaks once it is wide and the head turns. Instead sample the wall surface
	// (flat plane or cylinder, matching the warp) on a grid: the wall is "in
	// view" if ANY sample falls inside the frustum. Light up only when none do,
	// and point toward the wall centre.
	wp.offscreen_dir[0] = 0.0f;
	wp.offscreen_dir[1] = 0.0f;
	wp.offscreen_intensity = 0.0f;
	// Fixed-pixel glow thickness (so horizontal/vertical edges match), expressed
	// in centered-coord units: 2*px / dimension (centered coords span 2).
	const float offscreen_band_px = 50.0f;
	wp.offscreen_band[0] = eyes.eye_w
				       ? 2.0f * offscreen_band_px /
						 static_cast<float>(eyes.eye_w)
				       : 0.1f;
	wp.offscreen_band[1] = out_h ? 2.0f * offscreen_band_px /
					       static_cast<float>(out_h)
				     : 0.1f;
	if (wp.pose_valid > 0.5f &&
	    g_device.offscreen_indicator.load(std::memory_order_relaxed)) {
		const quatd qi = quat_inverse(q);
		const double tx = std::max(1e-4, (double)wp.tan_half_fov[0]);
		const double ty = std::max(1e-4, (double)wp.tan_half_fov[1]);
		const double W = wp.screen_half_size_m[0];
		const double H = wp.screen_half_size_m[1];
		const double dist = std::max(1e-3, (double)wp.screen_distance_m);
		const double curve = wp.screen_curve;
		const int NX = 9, NY = 5; // grid spacing < a frustum for realistic walls
		bool visible = false;
		double cdx = 0.0, cdy = 0.0; // wall-centre direction in view (the arrow)
		bool center_front = false;
		for (int iy = 0; iy < NY; ++iy) {
			for (int ix = 0; ix < NX; ++ix) {
				// lx = lateral position on the wall (arc length when curved).
				const double lx =
					((double)ix / (NX - 1) - 0.5) * 2.0 * W;
				const double ly =
					((double)iy / (NY - 1) - 0.5) * 2.0 * H;
				double px, pz;
				if (curve <= 1e-4) {
					px = lx;
					pz = -dist;
				} else {
					const double R = dist / curve;
					const double th = lx / R;
					px = R * std::sin(th);
					pz = (R - dist) - R * std::cos(th);
				}
				// Direction from the (gaze-dollied) eye to the wall
				// point - the remote's distance strip moves the
				// viewer, so it must be subtracted here too.
				const double rx = px - wp.viewer.x;
				const double ry = ly - wp.viewer.y;
				const double rz = pz - wp.viewer.z;
				const double pl =
					std::sqrt(rx * rx + ry * ry + rz * rz);
				if (pl < 1e-6)
					continue;
				const vec3d vd = rotate_vector(
					qi, {rx / pl, ry / pl, rz / pl});
				const bool is_center = (ix == NX / 2 && iy == NY / 2);
				if (vd.z < -1e-4) {
					const double nx = (vd.x / -vd.z) / tx;
					const double ny = (vd.y / -vd.z) / ty;
					if (std::fabs(nx) <= 1.0 &&
					    std::fabs(ny) <= 1.0)
						visible = true;
					if (is_center) {
						cdx = nx;
						cdy = ny;
						center_front = true;
					}
				}
			}
		}
		if (!visible) {
			// Constant strength (no distance falloff): the indicator
			// only conveys direction.
			double dx, dy;
			const double inten = 1.0;
			if (center_front) {
				const double l =
					std::sqrt(cdx * cdx + cdy * cdy) + 1e-9;
				dx = cdx / l;
				dy = cdy / l;
			} else { // wall centre is behind: point to the nearer side
				const vec3d vc = rotate_vector(
					qi, {0.0 - wp.viewer.x, 0.0 - wp.viewer.y,
					     -dist - wp.viewer.z});
				const double l = std::sqrt(vc.x * vc.x +
							   vc.y * vc.y);
				if (l < 1e-6) {
					dx = 1.0;
					dy = 0.0;
				} else {
					dx = vc.x / l;
					dy = vc.y / l;
				}
			}
			wp.offscreen_dir[0] = static_cast<float>(dx);
			wp.offscreen_dir[1] = static_cast<float>(dy);
			wp.offscreen_intensity = static_cast<float>(inten);
		}
	}
	return wp;
}
