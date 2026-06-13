// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "wall_layout.h"

#include <algorithm>
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
	int padding)
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
