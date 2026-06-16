// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "wall_layout.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <vector>

namespace {

uint32_t row_width(const std::vector<monitor_entry> &row, int gap_x)
{
	uint64_t width = 0;
	for (const monitor_entry &monitor : row)
		width += monitor.width;
	if (row.size() > 1)
		width += static_cast<uint64_t>(gap_x) * (row.size() - 1);
	return static_cast<uint32_t>(std::min<uint64_t>(width, UINT32_MAX));
}

uint32_t row_height(const std::vector<monitor_entry> &row)
{
	uint32_t height = 0;
	for (const monitor_entry &monitor : row)
		height = std::max(height, monitor.height);
	return height;
}

std::vector<LONG> sorted_unique_starts(const std::vector<monitor_entry> &monitors,
				       bool horizontal)
{
	std::vector<LONG> starts;
	starts.reserve(monitors.size());
	for (const monitor_entry &monitor : monitors)
		starts.push_back(horizontal ? monitor.x : monitor.y);

	std::sort(starts.begin(), starts.end());
	starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
	return starts;
}

int64_t windows_gap_offset(const std::vector<LONG> &starts, LONG position, int gap)
{
	if (gap <= 0 || starts.empty())
		return 0;

	const auto it = std::lower_bound(starts.begin(), starts.end(), position);
	return static_cast<int64_t>(std::distance(starts.begin(), it)) * gap;
}

} // namespace

wall_layout_result compute_rows_layout(
	const std::vector<std::vector<monitor_entry>> &rows, int gap_x,
	int gap_y, int padding, row_align align)
{
	wall_layout_result out;
	uint32_t content_width = 0;
	uint64_t content_height = 0;
	std::vector<uint32_t> row_widths;
	std::vector<uint32_t> row_heights;
	row_widths.reserve(rows.size());
	row_heights.reserve(rows.size());

	for (size_t row_index = 0; row_index < rows.size(); ++row_index) {
		const auto &row = rows[row_index];
		const uint32_t width = row_width(row, gap_x);
		const uint32_t height = row_height(row);
		row_widths.push_back(width);
		row_heights.push_back(height);
		content_width = std::max(content_width, width);
		content_height += height;
		if (row_index > 0)
			content_height += static_cast<uint64_t>(gap_y);
		for (const monitor_entry &monitor : row)
			out.monitors.push_back(monitor);
	}

	out.placements.reserve(out.monitors.size());
	int y = padding;
	for (size_t row_index = 0; row_index < rows.size(); ++row_index) {
		const auto &row = rows[row_index];
		int x = padding;
		const uint32_t remaining =
			content_width > row_widths[row_index]
				? content_width - row_widths[row_index]
				: 0;
		if (align == row_align::center)
			x += static_cast<int>(remaining / 2);
		else if (align == row_align::right)
			x += static_cast<int>(remaining);

		const uint32_t height = row_heights[row_index];
		for (const monitor_entry &monitor : row) {
			wall_placement p;
			p.x = x;
			p.y = y + static_cast<int>((height - monitor.height) / 2);
			p.width = monitor.width;
			p.height = monitor.height;
			out.placements.push_back(p);
			x += static_cast<int>(monitor.width) + gap_x;
		}
		y += static_cast<int>(height) + gap_y;
	}

	out.width = content_width + static_cast<uint32_t>(padding * 2);
	out.height = static_cast<uint32_t>(std::min<uint64_t>(
		content_height + static_cast<uint64_t>(padding * 2), UINT32_MAX));
	return out;
}

wall_layout_result compute_windows_layout(
	const std::vector<monitor_entry> &monitors, int gap_x, int gap_y,
	int padding, const std::vector<monitor_entry> &excluded)
{
	wall_layout_result out;
	out.monitors = monitors;

	if (monitors.empty()) {
		out.width = static_cast<uint32_t>(padding * 2);
		out.height = static_cast<uint32_t>(padding * 2);
		return out;
	}

	const std::vector<LONG> column_starts = sorted_unique_starts(monitors, true);
	const std::vector<LONG> row_starts = sorted_unique_starts(monitors, false);

	LONG min_x = monitors[0].x;
	LONG min_y = monitors[0].y;
	for (const monitor_entry &monitor : monitors) {
		min_x = std::min(min_x, monitor.x);
		min_y = std::min(min_y, monitor.y);
	}

	int64_t max_right = 0;
	int64_t max_bottom = 0;

	out.placements.reserve(monitors.size());
	for (const monitor_entry &monitor : monitors) {
		const int64_t x =
			static_cast<int64_t>(monitor.x - min_x) +
			windows_gap_offset(column_starts, monitor.x, gap_x);
		const int64_t y =
			static_cast<int64_t>(monitor.y - min_y) +
			windows_gap_offset(row_starts, monitor.y, gap_y);
		wall_placement p;
		p.x = padding + static_cast<int>(
				       std::min<int64_t>(x, INT32_MAX - padding));
		p.y = padding + static_cast<int>(
				       std::min<int64_t>(y, INT32_MAX - padding));
		p.width = monitor.width;
		p.height = monitor.height;
		out.placements.push_back(p);
		max_right = std::max(max_right, x + monitor.width);
		max_bottom = std::max(max_bottom, y + monitor.height);
	}

	// Holes: only a display *sandwiched* between wall monitors leaves a gap the
	// layout has to reserve. "Sandwiched" = a wall monitor to its left and
	// another to its right (sharing rows), or one above and one below (sharing
	// columns). A display merely at the edge has wall monitors on one side
	// only, so removing it just shrinks the wall - no interior hole, no mark.
	// When it is sandwiched, the same gap-offset math lands the placeholder
	// exactly in the gap its absence opened.
	for (const monitor_entry &e : excluded) {
		const LONG el = e.x;
		const LONG er = e.x + static_cast<LONG>(e.width);
		const LONG et = e.y;
		const LONG eb = e.y + static_cast<LONG>(e.height);
		const LONG ecx = e.x + static_cast<LONG>(e.width) / 2;
		const LONG ecy = e.y + static_cast<LONG>(e.height) / 2;
		bool left = false, right = false, above = false, below = false;
		for (const monitor_entry &m : monitors) {
			const LONG mcx = m.x + static_cast<LONG>(m.width) / 2;
			const LONG mcy = m.y + static_cast<LONG>(m.height) / 2;
			if (et < m.y + static_cast<LONG>(m.height) && m.y < eb) {
				if (mcx < ecx)
					left = true;
				else if (mcx > ecx)
					right = true;
			}
			if (el < m.x + static_cast<LONG>(m.width) && m.x < er) {
				if (mcy < ecy)
					above = true;
				else if (mcy > ecy)
					below = true;
			}
		}
		if (!((left && right) || (above && below)))
			continue;
		const int64_t x = static_cast<int64_t>(e.x - min_x) +
				  windows_gap_offset(column_starts, e.x, gap_x);
		const int64_t y = static_cast<int64_t>(e.y - min_y) +
				  windows_gap_offset(row_starts, e.y, gap_y);
		wall_placement p;
		p.x = padding +
		      static_cast<int>(std::min<int64_t>(x, INT32_MAX - padding));
		p.y = padding +
		      static_cast<int>(std::min<int64_t>(y, INT32_MAX - padding));
		p.width = e.width;
		p.height = e.height;
		out.blocked.push_back(p);
	}

	out.width = static_cast<uint32_t>(std::min<uint64_t>(
		static_cast<uint64_t>(std::max<int64_t>(0, max_right)) +
			static_cast<uint64_t>(padding * 2),
		UINT32_MAX));
	out.height = static_cast<uint32_t>(std::min<uint64_t>(
		static_cast<uint64_t>(std::max<int64_t>(0, max_bottom)) +
			static_cast<uint64_t>(padding * 2),
		UINT32_MAX));
	return out;
}

wall_audio_map compute_wall_audio_map(const wall_layout_result &layout,
				      const std::string &center_display_id)
{
	wall_audio_map out;
	if (layout.width == 0)
		return out;
	const float w = static_cast<float>(layout.width);
	const size_t n =
		std::min(layout.monitors.size(), layout.placements.size());
	for (size_t i = 0; i < n; i++) {
		const monitor_entry &mon = layout.monitors[i];
		const wall_placement &pl = layout.placements[i];
		if (pl.width == 0)
			continue;
		wall_monitor_map m;
		m.desk_left = mon.x;
		m.desk_right = mon.x + static_cast<long>(mon.width);
		m.u_left = static_cast<float>(pl.x) / w;
		m.u_right = static_cast<float>(pl.x) / w +
			    static_cast<float>(pl.width) / w;
		out.monitors.push_back(m);
		// A center display that is not part of the wall (or not
		// connected) stays -1 and behaves like auto.
		if (!center_display_id.empty() && mon.id == center_display_id)
			out.center_u = 0.5f * (m.u_left + m.u_right);
	}
	return out;
}

bool wall_u_from_desktop_x(const std::vector<wall_monitor_map> &map, double x,
			   double *u_out)
{
	if (map.empty())
		return false;
	const wall_monitor_map *nearest = nullptr;
	double nearest_dist = 0.0;
	for (const wall_monitor_map &m : map) {
		if (m.desk_right <= m.desk_left)
			continue;
		const double d = x < m.desk_left
					 ? m.desk_left - x
					 : (x > m.desk_right ? x - m.desk_right
							     : 0.0);
		if (!nearest || d < nearest_dist) {
			nearest = &m;
			nearest_dist = d;
		}
		if (d == 0.0)
			break;
	}
	if (!nearest)
		return false;
	const double t = (x - nearest->desk_left) /
			 (nearest->desk_right - nearest->desk_left);
	*u_out = nearest->u_left + t * (nearest->u_right - nearest->u_left);
	return true;
}

wall_rgba_bitmap wall_keepout_fill(int width, int height)
{
	wall_rgba_bitmap out;
	if (width < 1)
		width = 1;
	if (height < 1)
		height = 1;
	out.width = width;
	out.height = height;
	out.rgba.resize(static_cast<size_t>(width) * height * 4);

	// Zebra: equal-width 45-degree stripes (ISO 3864), white on black, framed
	// like a road diversion zone (導流帯). Drawn 1:1 so the angle stays 45 deg.
	const double period = 88.0;          // one stripe's width in px (band == gap)
	const uint8_t black[3] = {0, 0, 0};
	const uint8_t white[3] = {228, 228, 234};
	const int frame = std::clamp(std::min(width, height) / 40, 3, 14);
	const uint8_t fr[3] = {210, 210, 218};

	// Prohibited sign, painted opaquely on top (so the stripes never show
	// through it - that clash is what looked wrong before). Centered, round.
	const double cx = (width - 1) * 0.5;
	const double cy = (height - 1) * 0.5;
	const double span = std::min(width, height);
	const double r_out = span * 0.26;     // ring outer radius
	const double th = span * 0.066;       // ring + slash thickness
	const double r_mid = r_out - th * 0.5;
	const uint8_t red[3] = {221, 46, 68}; // #dd2e44

	// The whole placeholder is kept semi-transparent so it reads as a hint:
	// a uniform alpha means the baked sign and zebra dim together (no second
	// blend pass fighting the stripes).
	const uint8_t alpha = 50; // ~0.20, a faint hint over the black void

	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			// Zebra / frame base color.
			const uint8_t *base;
			if (x < frame || x >= width - frame || y < frame ||
			    y >= height - frame) {
				base = fr;
			} else {
				const long long cell = static_cast<long long>(
					std::floor((x + y) / period));
				base = (cell & 1) ? white : black;
			}
			// Sign coverage (ring + diagonal slash) painted over it.
			const double dx = x - cx;
			const double dy = y - cy;
			const double dist = std::sqrt(dx * dx + dy * dy);
			const double ring = std::clamp(
				0.5 - (std::fabs(dist - r_mid) - th * 0.5), 0.0,
				1.0);
			const double pd = std::fabs(dx - dy) * 0.70710678;
			double slash = std::clamp(0.5 - (pd - th * 0.5), 0.0, 1.0);
			slash = std::min(
				slash, std::clamp(0.5 - (dist - r_out), 0.0, 1.0));
			const double sign = std::max(ring, slash);
			const size_t i =
				(static_cast<size_t>(y) * width + x) * 4;
			out.rgba[i + 0] = static_cast<uint8_t>(std::lround(
				base[0] + (red[0] - base[0]) * sign));
			out.rgba[i + 1] = static_cast<uint8_t>(std::lround(
				base[1] + (red[1] - base[1]) * sign));
			out.rgba[i + 2] = static_cast<uint8_t>(std::lround(
				base[2] + (red[2] - base[2]) * sign));
			out.rgba[i + 3] = alpha;
		}
	}
	return out;
}
