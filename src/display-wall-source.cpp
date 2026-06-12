// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>

#include "audio-wall-source.h"
#include "display-wall-source.h"
#include "tooltip_util.h"

#include <obs-module.h>
#include <obs-frontend-api.h>
#include <graphics/graphics.h>
#include <util/config-file.h>
#include <util/platform.h>

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <climits>
#include <mutex>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

constexpr const char *DISPLAY_WALL_ID = "nyan_real_display_wall";
constexpr const char *MONITOR_CAPTURE_ID = "monitor_capture";
constexpr float DISPLAY_REFRESH_INTERVAL_S = 1.0f;
constexpr float OUTPUT_SIZE_SYNC_DELAY_S = 1.5f;

enum class layout_mode : int {
	auto_columns = 0,
	rows = 1,
	windows = 2,
};

enum class row_align : int {
	left = 0,
	center = 1,
	right = 2,
};

struct monitor_entry {
	int index = 0;
	int windows_number = 0;
	std::string id;
	std::string alt_id;
	std::string friendly_name;
	std::string device_name;
	std::string label;
	LONG x = 0;
	LONG y = 0;
	uint32_t width = 0;
	uint32_t height = 0;
	bool primary = false;
	uint16_t edid_vendor = 0;  // raw DISPLAYCONFIG edidManufactureId
	uint16_t edid_product = 0; // raw DISPLAYCONFIG edidProductCodeId
};

struct wall_child {
	monitor_entry monitor;
	obs_source_t *source = nullptr;
	int x = 0;
	int y = 0;
	uint32_t width = 0;
	uint32_t height = 0;
};

struct display_wall_source {
	obs_source_t *context = nullptr;
	std::vector<wall_child> children;
	bool active_children = false;
	// Spatial audio (Audio Wall) engine, alive while the checkable
	// "audio_wall" property group is on.
	audio_wall_engine *audio = nullptr;

	layout_mode mode = layout_mode::windows;
	row_align align = row_align::center;
	int columns = 3;
	int gap_x = 1;
	int gap_y = 1;
	int padding = 0;
	bool include_primary = true;
	bool exclude_glasses = true;
	bool capture_cursor = true;
	bool force_sdr = false;
	bool sync_output_size = false;
	std::string name_filter;
	std::string exclude_filter;
	std::string row_layout;
	std::string monitor_signature; // topology the last rebuild was built from
	float refresh_timer_s = 0.0f;
	float output_sync_timer_s = -1.0f;
	bool output_sync_queued = false;
	uint32_t pending_sync_width = 0;
	uint32_t pending_sync_height = 0;
	uint32_t last_synced_width = 0;
	uint32_t last_synced_height = 0;

	uint32_t width = 0;
	uint32_t height = 0;
};

struct output_size_sync_task {
	display_wall_source *wall = nullptr;
	obs_source_t *source = nullptr;
};

struct fit_scene_items_data {
	obs_source_t *source = nullptr;
	uint32_t width = 0;
	uint32_t height = 0;
	size_t updated = 0;
};

static std::string wide_to_utf8(const wchar_t *value)
{
	if (!value || !*value)
		return {};

	const int required =
		WideCharToMultiByte(CP_UTF8, 0, value, -1, nullptr, 0, nullptr, nullptr);
	if (required <= 1)
		return {};

	std::string out(static_cast<size_t>(required), '\0');
	WideCharToMultiByte(CP_UTF8, 0, value, -1, out.data(), required, nullptr,
			    nullptr);
	if (!out.empty() && out.back() == '\0')
		out.pop_back();
	return out;
}

static bool get_monitor_target_name(const wchar_t *device,
				    DISPLAYCONFIG_TARGET_DEVICE_NAME *target)
{
	UINT32 path_count = 0;
	UINT32 mode_count = 0;
	if (GetDisplayConfigBufferSizes(QDC_ONLY_ACTIVE_PATHS, &path_count,
					&mode_count) != ERROR_SUCCESS ||
	    path_count == 0 || mode_count == 0)
		return false;

	std::vector<DISPLAYCONFIG_PATH_INFO> paths(path_count);
	std::vector<DISPLAYCONFIG_MODE_INFO> modes(mode_count);
	if (QueryDisplayConfig(QDC_ONLY_ACTIVE_PATHS, &path_count, paths.data(),
			       &mode_count, modes.data(), nullptr) != ERROR_SUCCESS)
		return false;

	for (UINT32 i = 0; i < path_count; ++i) {
		const DISPLAYCONFIG_PATH_INFO &path = paths[i];
		DISPLAYCONFIG_SOURCE_DEVICE_NAME source = {};
		source.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_SOURCE_NAME;
		source.header.size = sizeof(source);
		source.header.adapterId = path.sourceInfo.adapterId;
		source.header.id = path.sourceInfo.id;
		if (DisplayConfigGetDeviceInfo(&source.header) != ERROR_SUCCESS ||
		    wcscmp(device, source.viewGdiDeviceName) != 0)
			continue;

		target->header.type = DISPLAYCONFIG_DEVICE_INFO_GET_TARGET_NAME;
		target->header.size = sizeof(*target);
		target->header.adapterId = path.sourceInfo.adapterId;
		target->header.id = path.targetInfo.id;
		return DisplayConfigGetDeviceInfo(&target->header) == ERROR_SUCCESS;
	}

	return false;
}

struct display_config_entry {
	std::string gdi_device;
	int windows_number = 0;
	std::string friendly_name;
	uint16_t edid_vendor = 0;  // raw DISPLAYCONFIG edidManufactureId
	uint16_t edid_product = 0; // raw DISPLAYCONFIG edidProductCodeId
};

static std::vector<display_config_entry> get_active_display_config_entries()
{
	UINT32 path_count = 0;
	UINT32 mode_count = 0;
	if (GetDisplayConfigBufferSizes(QDC_ONLY_ACTIVE_PATHS, &path_count,
					&mode_count) != ERROR_SUCCESS ||
	    path_count == 0 || mode_count == 0)
		return {};

	std::vector<DISPLAYCONFIG_PATH_INFO> paths(path_count);
	std::vector<DISPLAYCONFIG_MODE_INFO> modes(mode_count);
	if (QueryDisplayConfig(QDC_ONLY_ACTIVE_PATHS, &path_count, paths.data(),
			       &mode_count, modes.data(), nullptr) != ERROR_SUCCESS)
		return {};

	std::vector<display_config_entry> entries;
	entries.reserve(path_count);
	for (UINT32 i = 0; i < path_count; ++i) {
		const DISPLAYCONFIG_PATH_INFO &path = paths[i];
		DISPLAYCONFIG_SOURCE_DEVICE_NAME source = {};
		source.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_SOURCE_NAME;
		source.header.size = sizeof(source);
		source.header.adapterId = path.sourceInfo.adapterId;
		source.header.id = path.sourceInfo.id;
		if (DisplayConfigGetDeviceInfo(&source.header) != ERROR_SUCCESS)
			continue;

		display_config_entry entry;
		entry.gdi_device = wide_to_utf8(source.viewGdiDeviceName);
		entry.windows_number = static_cast<int>(i + 1);

		DISPLAYCONFIG_TARGET_DEVICE_NAME target = {};
		target.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_TARGET_NAME;
		target.header.size = sizeof(target);
		target.header.adapterId = path.targetInfo.adapterId;
		target.header.id = path.targetInfo.id;
		if (DisplayConfigGetDeviceInfo(&target.header) == ERROR_SUCCESS) {
			entry.friendly_name =
				wide_to_utf8(target.monitorFriendlyDeviceName);
			if (target.flags.edidIdsValid) {
				entry.edid_vendor = target.edidManufactureId;
				entry.edid_product = target.edidProductCodeId;
			}
		}

		if (!entry.gdi_device.empty())
			entries.push_back(std::move(entry));
	}
	return entries;
}

static const display_config_entry *find_display_config_entry(
	const std::vector<display_config_entry> &entries, const std::string &gdi_device)
{
	for (const display_config_entry &entry : entries) {
		if (entry.gdi_device == gdi_device)
			return &entry;
	}
	return nullptr;
}

static std::string get_friendly_monitor_name(HMONITOR handle)
{
	MONITORINFOEXW mi = {};
	mi.cbSize = sizeof(mi);
	DISPLAYCONFIG_TARGET_DEVICE_NAME target = {};
	if (GetMonitorInfoW(handle, reinterpret_cast<LPMONITORINFO>(&mi)) &&
	    get_monitor_target_name(mi.szDevice, &target)) {
		std::string name = wide_to_utf8(target.monitorFriendlyDeviceName);
		if (!name.empty())
			return name;
	}
	return {};
}

static std::string to_lower(std::string value)
{
	std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
		return static_cast<char>(std::tolower(ch));
	});
	return value;
}

static std::string trim_copy(const std::string &value)
{
	size_t begin = 0;
	while (begin < value.size() &&
	       std::isspace(static_cast<unsigned char>(value[begin])))
		++begin;

	size_t end = value.size();
	while (end > begin &&
	       std::isspace(static_cast<unsigned char>(value[end - 1])))
		--end;

	return value.substr(begin, end - begin);
}

static std::vector<std::string> split_filter_tokens(const std::string &text)
{
	std::vector<std::string> tokens;
	std::string token;
	for (char ch : text) {
		if (ch == ',' || ch == ';' || ch == '|' || ch == '\n' || ch == '\r') {
			std::string trimmed = trim_copy(token);
			if (!trimmed.empty())
				tokens.push_back(to_lower(trimmed));
			token.clear();
			continue;
		}
		token.push_back(ch);
	}
	std::string trimmed = trim_copy(token);
	if (!trimmed.empty())
		tokens.push_back(to_lower(trimmed));
	return tokens;
}

static std::string searchable_text(const monitor_entry &monitor)
{
	std::string text = monitor.label + " " + monitor.id + " " + monitor.alt_id +
			   " " + monitor.friendly_name + " " + monitor.device_name;
	return to_lower(text);
}

// --- glasses display identification (EDID) ---------------------------------

static std::vector<nyan_real_glasses_display_id> g_glasses_display_ids;

// DISPLAYCONFIG reports edidManufactureId as the raw little-endian word of
// EDID bytes 8..9, while the PNP encoding reads them big-endian; accept both
// orders so the comparison never depends on that quirk.
static bool vendor_word_matches(uint16_t edid_value, uint16_t expected_be)
{
	const uint16_t swapped = static_cast<uint16_t>(
		(edid_value >> 8) | static_cast<uint16_t>(edid_value << 8));
	return edid_value == expected_be || swapped == expected_be;
}

static bool is_glasses_display(uint16_t edid_vendor, uint16_t edid_product,
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

static bool parse_int_token(const std::string &token, int *value)
{
	if (token.empty())
		return false;
	char *end = nullptr;
	const long parsed = std::strtol(token.c_str(), &end, 10);
	if (!end || *end != '\0')
		return false;
	*value = static_cast<int>(parsed);
	return true;
}

static bool monitor_matches_token(const monitor_entry &monitor,
				  const std::string &token)
{
	int numeric = 0;
	if (parse_int_token(token, &numeric)) {
		if (monitor.windows_number > 0)
			return numeric == monitor.windows_number;
		return numeric == monitor.index || numeric == monitor.index + 1;
	}

	return searchable_text(monitor).find(token) != std::string::npos;
}

struct enum_monitor_context {
	std::vector<monitor_entry> monitors;
	std::vector<display_config_entry> config_entries;
};

static BOOL CALLBACK enum_display_monitor_proc(HMONITOR handle, HDC, LPRECT rect,
					       LPARAM param)
{
	auto *ctx = reinterpret_cast<enum_monitor_context *>(param);
	monitor_entry entry;
	entry.index = static_cast<int>(ctx->monitors.size());
	entry.x = rect->left;
	entry.y = rect->top;
	entry.width = static_cast<uint32_t>(std::max<LONG>(0, rect->right - rect->left));
	entry.height = static_cast<uint32_t>(std::max<LONG>(0, rect->bottom - rect->top));

	MONITORINFOEXA mi = {};
	mi.cbSize = sizeof(mi);
	if (GetMonitorInfoA(handle, reinterpret_cast<LPMONITORINFO>(&mi))) {
		entry.alt_id = mi.szDevice;
		entry.primary = (mi.dwFlags & MONITORINFOF_PRIMARY) != 0;
		if (const display_config_entry *config_entry =
			    find_display_config_entry(ctx->config_entries, entry.alt_id)) {
			entry.windows_number = config_entry->windows_number;
			entry.friendly_name = config_entry->friendly_name;
			entry.edid_vendor = config_entry->edid_vendor;
			entry.edid_product = config_entry->edid_product;
		}

		DISPLAY_DEVICEA device = {};
		device.cb = sizeof(device);
		if (EnumDisplayDevicesA(mi.szDevice, 0, &device,
					EDD_GET_DEVICE_INTERFACE_NAME)) {
			entry.id = device.DeviceID;
			entry.device_name = device.DeviceString;
		} else {
			entry.id = entry.alt_id;
		}
	}

	if (entry.friendly_name.empty())
		entry.friendly_name = get_friendly_monitor_name(handle);
	if (entry.friendly_name.empty())
		entry.friendly_name = entry.device_name.empty() ? entry.alt_id
								: entry.device_name;

	std::ostringstream label;
	const int display_number =
		entry.windows_number > 0 ? entry.windows_number : entry.index + 1;
	label << display_number << ": ";
	if (!entry.friendly_name.empty())
		label << entry.friendly_name;
	else
		label << entry.alt_id;
	label << " (" << entry.width << "x" << entry.height << " @ " << entry.x
	      << "," << entry.y << ")";
	if (entry.primary)
		label << " [primary]";
	entry.label = label.str();

	ctx->monitors.push_back(std::move(entry));
	return TRUE;
}

static BOOL CALLBACK monitor_signature_proc(HMONITOR handle, HDC, LPRECT rect,
					    LPARAM param)
{
	auto *sig = reinterpret_cast<std::ostringstream *>(param);
	*sig << rect->left << ',' << rect->top << ',' << rect->right << ','
	     << rect->bottom << ',';
	MONITORINFOEXW mi = {};
	mi.cbSize = sizeof(mi);
	if (GetMonitorInfoW(handle, reinterpret_cast<LPMONITORINFO>(&mi)))
		*sig << wide_to_utf8(mi.szDevice) << ','
		     << ((mi.dwFlags & MONITORINFOF_PRIMARY) ? 1 : 0);
	*sig << ';';
	return TRUE;
}

// Cheap monitor-topology snapshot for periodic change detection. The full
// enumerate_monitors() path resolves Windows display numbers and friendly
// names through QueryDisplayConfig and EnumDisplayDevices, which can block for
// milliseconds; running that every second inside video_tick stalls the video
// thread. This walks only EnumDisplayMonitors/GetMonitorInfo, so the periodic
// tick stays cheap until a display is added, removed, moved, or resized.
static std::string monitor_topology_signature()
{
	std::ostringstream sig;
	EnumDisplayMonitors(nullptr, nullptr, monitor_signature_proc,
			    reinterpret_cast<LPARAM>(&sig));
	return sig.str();
}

static std::vector<monitor_entry> enumerate_monitors()
{
	enum_monitor_context ctx;
	ctx.config_entries = get_active_display_config_entries();
	EnumDisplayMonitors(nullptr, nullptr, enum_display_monitor_proc,
			    reinterpret_cast<LPARAM>(&ctx));
	std::stable_sort(ctx.monitors.begin(), ctx.monitors.end(),
			 [](const monitor_entry &a, const monitor_entry &b) {
				 const int an = a.windows_number > 0
							? a.windows_number
							: std::numeric_limits<int>::max();
				 const int bn = b.windows_number > 0
							? b.windows_number
							: std::numeric_limits<int>::max();
				 if (an != bn)
					 return an < bn;
				 return a.index < b.index;
			 });
	return ctx.monitors;
}

static std::vector<monitor_entry> filter_monitors(const std::vector<monitor_entry> &all,
						  bool include_primary,
						  bool exclude_glasses,
						  const std::string &filter,
						  const std::string &exclude_filter)
{
	const std::vector<std::string> tokens = split_filter_tokens(filter);
	const std::vector<std::string> exclude_tokens =
		split_filter_tokens(exclude_filter);
	std::vector<monitor_entry> selected;

	for (const monitor_entry &monitor : all) {
		if (!include_primary && monitor.primary)
			continue;
		// Capturing the display the glasses show creates a feedback
		// loop through the source projector, so known glasses panels
		// are dropped before the user filters.
		if (exclude_glasses &&
		    is_glasses_display(monitor.edid_vendor, monitor.edid_product,
				       monitor.friendly_name))
			continue;

		bool excluded = false;
		for (const std::string &token : exclude_tokens) {
			if (!token.empty() && monitor_matches_token(monitor, token)) {
				excluded = true;
				break;
			}
		}
		if (excluded)
			continue;

		if (!tokens.empty()) {
			bool matched = false;
			for (const std::string &token : tokens) {
				if (!token.empty() && monitor_matches_token(monitor, token)) {
					matched = true;
					break;
				}
			}
			if (!matched)
				continue;
		}

		selected.push_back(monitor);
	}

	return selected;
}

static const monitor_entry *find_monitor_by_token(
	const std::vector<monitor_entry> &monitors, const std::string &token,
	bool zero_based_numbers)
{
	int numeric = 0;
	if (parse_int_token(token, &numeric)) {
		if (!zero_based_numbers) {
			for (const monitor_entry &monitor : monitors) {
				if (monitor.windows_number > 0 &&
				    monitor.windows_number == numeric)
					return &monitor;
			}
		}

		const int index = zero_based_numbers ? numeric : numeric - 1;
		if (index >= 0 && index < static_cast<int>(monitors.size()))
			return &monitors[static_cast<size_t>(index)];
		return nullptr;
	}

	const std::string needle = to_lower(token);
	for (const monitor_entry &monitor : monitors) {
		if (searchable_text(monitor).find(needle) != std::string::npos)
			return &monitor;
	}
	return nullptr;
}

static std::vector<std::vector<monitor_entry>>
build_rows_from_text(const std::vector<monitor_entry> &monitors,
		     const std::string &layout)
{
	std::vector<std::vector<std::string>> token_rows;
	std::vector<std::string> current_row;
	std::string token;
	bool has_zero = false;

	auto flush_token = [&]() {
		if (token.empty())
			return;
		int parsed = 0;
		if (parse_int_token(token, &parsed) && parsed == 0)
			has_zero = true;
		current_row.push_back(token);
		token.clear();
	};
	auto flush_row = [&]() {
		flush_token();
		if (!current_row.empty()) {
			token_rows.push_back(current_row);
			current_row.clear();
		}
	};

	for (char ch : layout) {
		if (ch == '/' || ch == ';' || ch == '\n' || ch == '\r') {
			flush_row();
		} else if (ch == ',' || std::isspace(static_cast<unsigned char>(ch))) {
			flush_token();
		} else {
			token.push_back(ch);
		}
	}
	flush_row();

	std::vector<std::vector<monitor_entry>> rows;
	std::unordered_set<std::string> used;
	for (const std::vector<std::string> &token_row : token_rows) {
		std::vector<monitor_entry> row;
		for (const std::string &item : token_row) {
			const monitor_entry *monitor =
				find_monitor_by_token(monitors, item, has_zero);
			if (!monitor || used.find(monitor->id) != used.end())
				continue;
			row.push_back(*monitor);
			used.insert(monitor->id);
		}
		if (!row.empty())
			rows.push_back(std::move(row));
	}
	return rows;
}

static std::vector<std::vector<monitor_entry>>
build_auto_rows(const std::vector<monitor_entry> &monitors, int columns)
{
	std::vector<std::vector<monitor_entry>> rows;
	const int safe_columns = std::max(1, columns);
	for (size_t i = 0; i < monitors.size();) {
		std::vector<monitor_entry> row;
		for (int col = 0; col < safe_columns && i < monitors.size(); ++col, ++i)
			row.push_back(monitors[i]);
		rows.push_back(std::move(row));
	}
	return rows;
}

static uint32_t row_width(const std::vector<monitor_entry> &row, int gap_x)
{
	uint64_t width = 0;
	for (const monitor_entry &monitor : row)
		width += monitor.width;
	if (row.size() > 1)
		width += static_cast<uint64_t>(gap_x) * (row.size() - 1);
	return static_cast<uint32_t>(std::min<uint64_t>(width, UINT32_MAX));
}

static uint32_t row_height(const std::vector<monitor_entry> &row)
{
	uint32_t height = 0;
	for (const monitor_entry &monitor : row)
		height = std::max(height, monitor.height);
	return height;
}

static std::vector<LONG> sorted_unique_starts(
	const std::vector<monitor_entry> &monitors, bool horizontal)
{
	std::vector<LONG> starts;
	starts.reserve(monitors.size());
	for (const monitor_entry &monitor : monitors)
		starts.push_back(horizontal ? monitor.x : monitor.y);

	std::sort(starts.begin(), starts.end());
	starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
	return starts;
}

static int64_t windows_gap_offset(const std::vector<LONG> &starts, LONG position,
				  int gap)
{
	if (gap <= 0 || starts.empty())
		return 0;

	const auto it = std::lower_bound(starts.begin(), starts.end(), position);
	return static_cast<int64_t>(std::distance(starts.begin(), it)) * gap;
}

static void remove_active_children(display_wall_source *wall)
{
	if (!wall->active_children)
		return;
	for (wall_child &child : wall->children) {
		if (child.source)
			obs_source_remove_active_child(wall->context, child.source);
	}
	wall->active_children = false;
}

static void add_active_children(display_wall_source *wall)
{
	if (wall->active_children || !obs_source_showing(wall->context))
		return;
	for (wall_child &child : wall->children) {
		if (child.source)
			obs_source_add_active_child(wall->context, child.source);
	}
	wall->active_children = true;
}

// ---- desktop -> wall-texture mapping for the Audio Wall --------------------
static std::mutex g_wall_map_mutex;
static std::vector<nyan_wall_monitor_map> g_wall_map;
static std::atomic<uint32_t> g_wall_map_gen{0};

static void publish_wall_audio_map(display_wall_source *wall)
{
	std::vector<nyan_wall_monitor_map> map;
	if (wall->width > 0) {
		const float w = static_cast<float>(wall->width);
		for (const wall_child &child : wall->children) {
			if (!child.source || child.width == 0)
				continue;
			nyan_wall_monitor_map m;
			m.desk_left = child.monitor.x;
			m.desk_right = child.monitor.x +
				       static_cast<long>(child.monitor.width);
			m.u_left = static_cast<float>(child.x) / w;
			m.u_right = static_cast<float>(child.x) / w +
				    static_cast<float>(child.width) / w;
			map.push_back(m);
		}
	}
	{
		std::lock_guard<std::mutex> lk(g_wall_map_mutex);
		g_wall_map = std::move(map);
	}
	g_wall_map_gen.fetch_add(1, std::memory_order_relaxed);
}

static obs_data_t *make_monitor_capture_settings(const monitor_entry &monitor,
						 bool capture_cursor,
						 bool force_sdr)
{
	obs_data_t *settings = obs_data_create();
	obs_data_set_int(settings, "method", 0);
	obs_data_set_string(settings, "monitor_id", monitor.id.c_str());
	obs_data_set_int(settings, "monitor", monitor.index);
	obs_data_set_bool(settings, "capture_cursor", capture_cursor);
	obs_data_set_bool(settings, "force_sdr", force_sdr);
	return settings;
}

static void release_children(display_wall_source *wall)
{
	remove_active_children(wall);
	for (wall_child &child : wall->children) {
		if (child.source)
			obs_source_release(child.source);
	}
	wall->children.clear();
}

static bool same_child_set(const std::vector<wall_child> &children,
			   const std::vector<monitor_entry> &monitors,
			   bool capture_cursor, bool force_sdr)
{
	if (children.size() != monitors.size())
		return false;

	for (size_t i = 0; i < monitors.size(); ++i) {
		if (!children[i].source)
			return false;
		if (children[i].monitor.id != monitors[i].id)
			return false;
		obs_data_t *settings =
			obs_source_get_settings(children[i].source);
		const bool same_cursor =
			obs_data_get_bool(settings, "capture_cursor") == capture_cursor;
		const bool same_sdr =
			obs_data_get_bool(settings, "force_sdr") == force_sdr;
		obs_data_release(settings);
		if (!same_cursor || !same_sdr)
			return false;
	}
	return true;
}

static void ensure_children(display_wall_source *wall,
			    const std::vector<monitor_entry> &monitors)
{
	if (!same_child_set(wall->children, monitors, wall->capture_cursor,
			    wall->force_sdr)) {
		release_children(wall);
		for (const monitor_entry &monitor : monitors) {
			wall_child child;
			child.monitor = monitor;
			child.width = monitor.width;
			child.height = monitor.height;
			obs_data_t *settings = make_monitor_capture_settings(
				monitor, wall->capture_cursor, wall->force_sdr);
			std::string name = "nyan Real Display Wall - " + monitor.label;
			child.source = obs_source_create_private(
				MONITOR_CAPTURE_ID, name.c_str(), settings);
			obs_data_release(settings);
			if (!child.source) {
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] Display Wall could not create private monitor_capture for %s",
				     monitor.label.c_str());
			}
			wall->children.push_back(std::move(child));
		}
	}
}

static void apply_layout(display_wall_source *wall,
			 const std::vector<std::vector<monitor_entry>> &rows)
{
	std::vector<monitor_entry> flat;
	uint32_t content_width = 0;
	uint64_t content_height = 0;
	std::vector<uint32_t> row_widths;
	std::vector<uint32_t> row_heights;
	row_widths.reserve(rows.size());
	row_heights.reserve(rows.size());

	for (size_t row_index = 0; row_index < rows.size(); ++row_index) {
		const auto &row = rows[row_index];
		const uint32_t width = row_width(row, wall->gap_x);
		const uint32_t height = row_height(row);
		row_widths.push_back(width);
		row_heights.push_back(height);
		content_width = std::max(content_width, width);
		content_height += height;
		if (row_index > 0)
			content_height += static_cast<uint64_t>(wall->gap_y);
		for (const monitor_entry &monitor : row)
			flat.push_back(monitor);
	}

	ensure_children(wall, flat);

	size_t child_index = 0;
	int y = wall->padding;
	for (size_t row_index = 0; row_index < rows.size(); ++row_index) {
		const auto &row = rows[row_index];
		int x = wall->padding;
		const uint32_t remaining =
			content_width > row_widths[row_index]
				? content_width - row_widths[row_index]
				: 0;
		if (wall->align == row_align::center)
			x += static_cast<int>(remaining / 2);
		else if (wall->align == row_align::right)
			x += static_cast<int>(remaining);

		const uint32_t height = row_heights[row_index];
		for (const monitor_entry &monitor : row) {
			if (child_index >= wall->children.size())
				break;
			wall_child &child = wall->children[child_index++];
			child.x = x;
			child.y = y + static_cast<int>((height - monitor.height) / 2);
			child.width = monitor.width;
			child.height = monitor.height;
			x += static_cast<int>(monitor.width) + wall->gap_x;
		}
		y += static_cast<int>(height) + wall->gap_y;
	}

	wall->width = content_width + static_cast<uint32_t>(wall->padding * 2);
	wall->height = static_cast<uint32_t>(
		std::min<uint64_t>(content_height + static_cast<uint64_t>(wall->padding * 2),
				   UINT32_MAX));

	publish_wall_audio_map(wall);
	add_active_children(wall);
}

static void apply_windows_layout(display_wall_source *wall,
				 const std::vector<monitor_entry> &monitors)
{
	ensure_children(wall, monitors);

	if (monitors.empty()) {
		wall->width = static_cast<uint32_t>(wall->padding * 2);
		wall->height = static_cast<uint32_t>(wall->padding * 2);
		publish_wall_audio_map(wall);
		add_active_children(wall);
		return;
	}

	const std::vector<LONG> column_starts =
		sorted_unique_starts(monitors, true);
	const std::vector<LONG> row_starts =
		sorted_unique_starts(monitors, false);

	LONG min_x = monitors[0].x;
	LONG min_y = monitors[0].y;
	for (const monitor_entry &monitor : monitors) {
		min_x = std::min(min_x, monitor.x);
		min_y = std::min(min_y, monitor.y);
	}

	int64_t max_right = 0;
	int64_t max_bottom = 0;

	for (size_t i = 0; i < monitors.size() && i < wall->children.size(); ++i) {
		const monitor_entry &monitor = monitors[i];
		wall_child &child = wall->children[i];
		const int64_t x =
			static_cast<int64_t>(monitor.x - min_x) +
			windows_gap_offset(column_starts, monitor.x, wall->gap_x);
		const int64_t y =
			static_cast<int64_t>(monitor.y - min_y) +
			windows_gap_offset(row_starts, monitor.y, wall->gap_y);
		child.x = wall->padding + static_cast<int>(std::min<int64_t>(
						  x, INT32_MAX - wall->padding));
		child.y = wall->padding + static_cast<int>(std::min<int64_t>(
						  y, INT32_MAX - wall->padding));
		child.width = monitor.width;
		child.height = monitor.height;
		max_right = std::max(max_right, x + monitor.width);
		max_bottom = std::max(max_bottom, y + monitor.height);
	}

	wall->width = static_cast<uint32_t>(std::min<uint64_t>(
		static_cast<uint64_t>(std::max<int64_t>(0, max_right)) +
			static_cast<uint64_t>(wall->padding * 2),
		UINT32_MAX));
	wall->height = static_cast<uint32_t>(std::min<uint64_t>(
		static_cast<uint64_t>(std::max<int64_t>(0, max_bottom)) +
			static_cast<uint64_t>(wall->padding * 2),
		UINT32_MAX));

	publish_wall_audio_map(wall);
	add_active_children(wall);
}

static bool obs_output_is_active()
{
	return obs_frontend_streaming_active() || obs_frontend_recording_active() ||
	       obs_frontend_replay_buffer_active() || obs_frontend_virtualcam_active();
}

static void schedule_output_size_sync(display_wall_source *wall)
{
	if (!wall->sync_output_size || wall->width < 32 || wall->height < 32)
		return;
	if (wall->last_synced_width == wall->width &&
	    wall->last_synced_height == wall->height)
		return;
	if (wall->output_sync_timer_s >= 0.0f &&
	    wall->pending_sync_width == wall->width &&
	    wall->pending_sync_height == wall->height)
		return;

	wall->pending_sync_width = wall->width;
	wall->pending_sync_height = wall->height;
	wall->output_sync_timer_s = OUTPUT_SIZE_SYNC_DELAY_S;
	blog(LOG_INFO,
	     "[obs-nyan-real-3dof] Display Wall scheduled OBS output size sync: %ux%u",
	     wall->width, wall->height);
}

static void release_output_size_sync_task(output_size_sync_task *task)
{
	if (!task)
		return;
	if (task->source)
		obs_source_release(task->source);
	delete task;
}

static bool fit_display_wall_scene_item(obs_scene_t *, obs_sceneitem_t *item,
					void *param)
{
	auto *data = static_cast<fit_scene_items_data *>(param);
	if (!data || !item)
		return true;

	if (obs_sceneitem_is_group(item))
		obs_sceneitem_group_enum_items(item, fit_display_wall_scene_item, param);

	if (obs_sceneitem_locked(item))
		return true;

	if (obs_sceneitem_get_source(item) != data->source)
		return true;

	obs_transform_info info = {};
	info.pos.x = 0.0f;
	info.pos.y = 0.0f;
	info.rot = 0.0f;
	info.scale.x = 1.0f;
	info.scale.y = 1.0f;
	info.alignment = OBS_ALIGN_LEFT | OBS_ALIGN_TOP;
	info.bounds_type = OBS_BOUNDS_SCALE_INNER;
	info.bounds_alignment = OBS_ALIGN_CENTER;
	info.bounds.x = static_cast<float>(data->width);
	info.bounds.y = static_cast<float>(data->height);
	info.crop_to_bounds = obs_sceneitem_get_bounds_crop(item);

	obs_sceneitem_set_info2(item, &info);
	++data->updated;
	return true;
}

static bool fit_display_wall_scene(obs_scene_t *scene,
				   fit_scene_items_data *data)
{
	if (!scene || !data)
		return true;

	obs_scene_enum_items(scene, fit_display_wall_scene_item, data);
	return true;
}

static bool fit_display_wall_scene_source(void *param, obs_source_t *source)
{
	if (!source)
		return true;

	obs_scene_t *scene = obs_scene_from_source(source);
	if (!scene)
		return true;

	return fit_display_wall_scene(scene,
				      static_cast<fit_scene_items_data *>(param));
}

static void fit_display_wall_scene_items(obs_source_t *source, uint32_t width,
					 uint32_t height)
{
	if (!source || width == 0 || height == 0)
		return;

	fit_scene_items_data data;
	data.source = source;
	data.width = width;
	data.height = height;
	obs_enum_scenes(fit_display_wall_scene_source, &data);

	if (data.updated > 0)
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] Display Wall fit %zu scene item(s) to OBS output size: %ux%u",
		     data.updated, width, height);
}

static void sync_obs_output_size_ui(void *param)
{
	auto *task = static_cast<output_size_sync_task *>(param);
	if (!task)
		return;

	display_wall_source *wall = task->wall;
	if (!wall || wall->context != task->source) {
		release_output_size_sync_task(task);
		return;
	}

	wall->output_sync_queued = false;

	obs_data_t *settings = obs_source_get_settings(task->source);
	const bool enabled =
		settings && obs_data_get_bool(settings, "sync_output_size");
	if (settings)
		obs_data_release(settings);

	const uint32_t width = wall->width;
	const uint32_t height = wall->height;
	if (!enabled || width < 32 || height < 32) {
		wall->output_sync_timer_s = -1.0f;
		wall->pending_sync_width = 0;
		wall->pending_sync_height = 0;
		release_output_size_sync_task(task);
		return;
	}

	if (obs_output_is_active()) {
		wall->output_sync_timer_s = OUTPUT_SIZE_SYNC_DELAY_S;
		wall->pending_sync_width = width;
		wall->pending_sync_height = height;
		release_output_size_sync_task(task);
		return;
	}

	struct obs_video_info ovi = {};
	if (obs_get_video_info(&ovi) && ovi.base_width == width &&
	    ovi.base_height == height && ovi.output_width == width &&
	    ovi.output_height == height) {
		fit_display_wall_scene_items(task->source, width, height);
		wall->last_synced_width = width;
		wall->last_synced_height = height;
		wall->output_sync_timer_s = -1.0f;
		wall->pending_sync_width = 0;
		wall->pending_sync_height = 0;
		release_output_size_sync_task(task);
		return;
	}

	config_t *profile = obs_frontend_get_profile_config();
	if (!profile) {
		wall->output_sync_timer_s = OUTPUT_SIZE_SYNC_DELAY_S;
		wall->pending_sync_width = width;
		wall->pending_sync_height = height;
		release_output_size_sync_task(task);
		return;
	}

	config_set_uint(profile, "Video", "BaseCX", width);
	config_set_uint(profile, "Video", "BaseCY", height);
	config_set_uint(profile, "Video", "OutputCX", width);
	config_set_uint(profile, "Video", "OutputCY", height);
	obs_frontend_reset_video();
	fit_display_wall_scene_items(task->source, width, height);
	obs_frontend_save();

	wall->last_synced_width = width;
	wall->last_synced_height = height;
	wall->output_sync_timer_s = -1.0f;
	wall->pending_sync_width = 0;
	wall->pending_sync_height = 0;
	blog(LOG_INFO,
	     "[obs-nyan-real-3dof] Display Wall matched OBS output size: %ux%u",
	     width, height);
	release_output_size_sync_task(task);
}

static void sync_obs_output_size(display_wall_source *wall)
{
	if (!wall->sync_output_size || wall->width < 32 || wall->height < 32)
		return;

	if (wall->output_sync_queued)
		return;

	obs_source_t *source = obs_source_get_ref(wall->context);
	if (!source) {
		wall->output_sync_timer_s = OUTPUT_SIZE_SYNC_DELAY_S;
		return;
	}

	auto *task = new output_size_sync_task();
	task->wall = wall;
	task->source = source;
	wall->output_sync_queued = true;
	wall->output_sync_timer_s = -1.0f;
	obs_queue_task(OBS_TASK_UI, sync_obs_output_size_ui, task, false);
}

static std::string build_monitor_info_text(const std::vector<monitor_entry> &all,
					   const std::vector<monitor_entry> &selected)
{
	std::ostringstream out;
	out << obs_module_text("display_wall.detected_prefix") << "\n";
	if (all.empty()) {
		out << "  " << obs_module_text("display_wall.no_displays");
		return out.str();
	}

	std::unordered_set<std::string> selected_ids;
	for (const monitor_entry &monitor : selected)
		selected_ids.insert(monitor.id);

	for (const monitor_entry &monitor : all) {
		out << (selected_ids.find(monitor.id) != selected_ids.end() ? "* " : "  ");
		out << monitor.label << "\n";
	}
	return out.str();
}

static void rebuild_display_wall(display_wall_source *wall)
{
	const uint32_t old_width = wall->width;
	const uint32_t old_height = wall->height;
	wall->monitor_signature = monitor_topology_signature();
	const std::vector<monitor_entry> all = enumerate_monitors();
	const std::vector<monitor_entry> selected =
		filter_monitors(all, wall->include_primary, wall->exclude_glasses,
				wall->name_filter, wall->exclude_filter);

	if (wall->mode == layout_mode::windows) {
		apply_windows_layout(wall, selected);
	} else {
		std::vector<std::vector<monitor_entry>> rows;
		if (wall->mode == layout_mode::rows && !wall->row_layout.empty())
			rows = build_rows_from_text(selected, wall->row_layout);
		if (rows.empty())
			rows = build_auto_rows(selected, wall->columns);

		apply_layout(wall, rows);
	}

	if (old_width != wall->width || old_height != wall->height ||
	    wall->last_synced_width != wall->width ||
	    wall->last_synced_height != wall->height)
		schedule_output_size_sync(wall);
}

static void display_wall_update(void *data, obs_data_t *settings)
{
	auto *wall = static_cast<display_wall_source *>(data);
	if (!settings)
		return;

	wall->mode = static_cast<layout_mode>(obs_data_get_int(settings, "layout_mode"));
	wall->align = static_cast<row_align>(obs_data_get_int(settings, "row_align"));
	wall->columns =
		std::max(1, static_cast<int>(obs_data_get_int(settings, "columns")));
	wall->gap_x = std::max(0, static_cast<int>(obs_data_get_int(settings, "gap_x")));
	wall->gap_y = std::max(0, static_cast<int>(obs_data_get_int(settings, "gap_y")));
	wall->padding =
		std::max(0, static_cast<int>(obs_data_get_int(settings, "padding")));
	wall->include_primary = obs_data_get_bool(settings, "include_primary");
	wall->exclude_glasses = obs_data_get_bool(settings, "exclude_glasses");
	wall->capture_cursor = obs_data_get_bool(settings, "capture_cursor");
	wall->force_sdr = obs_data_get_bool(settings, "force_sdr");
	const bool previous_sync_output_size = wall->sync_output_size;
	wall->sync_output_size = obs_data_get_bool(settings, "sync_output_size");
	const char *name_filter = obs_data_get_string(settings, "name_filter");
	wall->name_filter = name_filter ? name_filter : "";
	const char *exclude_filter = obs_data_get_string(settings, "exclude_filter");
	wall->exclude_filter = exclude_filter ? exclude_filter : "";
	const char *row_layout = obs_data_get_string(settings, "row_layout");
	wall->row_layout = row_layout ? row_layout : "";
	wall->refresh_timer_s = 0.0f;

	rebuild_display_wall(wall);
	if (!previous_sync_output_size && wall->sync_output_size) {
		wall->last_synced_width = 0;
		wall->last_synced_height = 0;
		schedule_output_size_sync(wall);
	}

	// Spatial audio engine follows the checkable group. The engine's tick
	// re-activates its children by itself, so no show forwarding is
	// needed on a mid-session enable.
	const bool want_audio = obs_data_get_bool(settings, "audio_wall");
	if (want_audio && !wall->audio)
		wall->audio = audio_wall_create(wall->context);
	else if (!want_audio && wall->audio) {
		audio_wall_destroy(wall->audio);
		wall->audio = nullptr;
	}
	if (wall->audio)
		audio_wall_update(wall->audio, settings);
}

static const char *display_wall_get_name(void *)
{
	return obs_module_text("display_wall.name");
}

static void *display_wall_create(obs_data_t *settings, obs_source_t *context)
{
	auto *wall = new display_wall_source();
	wall->context = context;
	display_wall_update(wall, settings);
	blog(LOG_INFO, "[obs-nyan-real-3dof] Display Wall source created");
	return wall;
}

static void display_wall_destroy(void *data)
{
	auto *wall = static_cast<display_wall_source *>(data);
	if (wall->audio)
		audio_wall_destroy(wall->audio);
	release_children(wall);
	delete wall;
}

static uint32_t display_wall_get_width(void *data)
{
	auto *wall = static_cast<display_wall_source *>(data);
	return wall->width;
}

static uint32_t display_wall_get_height(void *data)
{
	auto *wall = static_cast<display_wall_source *>(data);
	return wall->height;
}

static void display_wall_defaults(obs_data_t *settings)
{
	obs_data_set_default_int(settings, "layout_mode",
				 static_cast<int>(layout_mode::windows));
	obs_data_set_default_int(settings, "columns", 3);
	obs_data_set_default_int(settings, "row_align", static_cast<int>(row_align::center));
	obs_data_set_default_int(settings, "gap_x", 1);
	obs_data_set_default_int(settings, "gap_y", 1);
	obs_data_set_default_int(settings, "padding", 0);
	obs_data_set_default_bool(settings, "include_primary", true);
	obs_data_set_default_bool(settings, "exclude_glasses", true);
	obs_data_set_default_bool(settings, "capture_cursor", true);
	obs_data_set_default_bool(settings, "force_sdr", false);
	obs_data_set_default_bool(settings, "sync_output_size", false);
	obs_data_set_default_string(settings, "name_filter", "");
	obs_data_set_default_string(settings, "exclude_filter", "");
	obs_data_set_default_string(settings, "row_layout", "");
	audio_wall_defaults(settings);
}

static void set_property_visible(obs_properties_t *props, const char *name,
				 bool visible)
{
	obs_property_t *property = obs_properties_get(props, name);
	if (property)
		obs_property_set_visible(property, visible);
}

static bool display_wall_layout_mode_changed(obs_properties_t *props,
					     obs_property_t *,
					     obs_data_t *settings)
{
	const auto mode =
		static_cast<layout_mode>(obs_data_get_int(settings, "layout_mode"));
	set_property_visible(props, "columns", mode == layout_mode::auto_columns);
	set_property_visible(props, "row_layout", mode == layout_mode::rows);
	set_property_visible(props, "row_align", mode != layout_mode::windows);
	return true;
}

static obs_properties_t *display_wall_properties(void *data)
{
	auto *wall = static_cast<display_wall_source *>(data);
	const std::vector<monitor_entry> all = enumerate_monitors();

	bool include_primary = false;
	bool exclude_glasses = true;
	std::string name_filter;
	std::string exclude_filter;
	if (wall) {
		include_primary = wall->include_primary;
		exclude_glasses = wall->exclude_glasses;
		name_filter = wall->name_filter;
		exclude_filter = wall->exclude_filter;
	}

	const std::vector<monitor_entry> selected =
		filter_monitors(all, include_primary, exclude_glasses, name_filter,
				exclude_filter);

	obs_properties_t *props = obs_properties_create();
	obs_properties_add_text(props, "display_info",
				build_monitor_info_text(all, selected).c_str(),
				OBS_TEXT_INFO);

	obs_property_t *mode = obs_properties_add_list(
		props, "layout_mode", obs_module_text("display_wall.layout_mode"),
		OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
	obs_property_list_add_int(mode, obs_module_text("display_wall.layout_auto"),
				  static_cast<int>(layout_mode::auto_columns));
	obs_property_list_add_int(mode, obs_module_text("display_wall.layout_rows"),
				  static_cast<int>(layout_mode::rows));
	obs_property_list_add_int(mode, obs_module_text("display_wall.layout_windows"),
				  static_cast<int>(layout_mode::windows));
	obs_property_set_modified_callback(mode, display_wall_layout_mode_changed);

	obs_properties_add_int(props, "columns", obs_module_text("display_wall.columns"),
			       1, 16, 1);
	obs_property_t *rows = obs_properties_add_text(
		props, "row_layout", obs_module_text("display_wall.row_layout"),
		OBS_TEXT_MULTILINE);
	obs_property_set_long_description(
		rows, wrapped_tooltip("display_wall.row_layout_tooltip").c_str());

	obs_property_t *align = obs_properties_add_list(
		props, "row_align", obs_module_text("display_wall.row_align"),
		OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
	obs_property_list_add_int(align, obs_module_text("display_wall.align_left"),
				  static_cast<int>(row_align::left));
	obs_property_list_add_int(align, obs_module_text("display_wall.align_center"),
				  static_cast<int>(row_align::center));
	obs_property_list_add_int(align, obs_module_text("display_wall.align_right"),
				  static_cast<int>(row_align::right));

	obs_properties_add_int(props, "gap_x", obs_module_text("display_wall.gap_x"),
			       0, 4096, 1);
	obs_properties_add_int(props, "gap_y", obs_module_text("display_wall.gap_y"),
			       0, 4096, 1);
	obs_properties_add_int(props, "padding",
			       obs_module_text("display_wall.padding"), 0, 4096, 1);
	obs_properties_add_bool(props, "include_primary",
				obs_module_text("display_wall.include_primary"));
	obs_property_t *exclude_glasses_prop = obs_properties_add_bool(
		props, "exclude_glasses",
		obs_module_text("display_wall.exclude_glasses"));
	obs_property_set_long_description(
		exclude_glasses_prop,
		wrapped_tooltip("display_wall.exclude_glasses_tooltip").c_str());
	obs_property_t *sync_output = obs_properties_add_bool(
		props, "sync_output_size",
		obs_module_text("display_wall.sync_output_size"));
	obs_property_set_long_description(
		sync_output,
		wrapped_tooltip("display_wall.sync_output_size_tooltip").c_str());
	obs_property_t *filter = obs_properties_add_text(
		props, "name_filter", obs_module_text("display_wall.name_filter"),
		OBS_TEXT_DEFAULT);
	obs_property_set_long_description(
		filter, wrapped_tooltip("display_wall.name_filter_tooltip").c_str());
	obs_property_t *exclude = obs_properties_add_text(
		props, "exclude_filter", obs_module_text("display_wall.exclude_filter"),
		OBS_TEXT_DEFAULT);
	obs_property_set_long_description(
		exclude,
		wrapped_tooltip("display_wall.exclude_filter_tooltip").c_str());
	obs_properties_add_bool(props, "capture_cursor",
				obs_module_text("display_wall.capture_cursor"));
	obs_properties_add_bool(props, "force_sdr",
				obs_module_text("display_wall.force_sdr"));

	audio_wall_add_properties(wall ? wall->audio : nullptr, props);

	if (wall) {
		obs_data_t *current = obs_source_get_settings(wall->context);
		display_wall_layout_mode_changed(props, nullptr, current);
		obs_data_release(current);
	}

	return props;
}

static void display_wall_render(void *data, gs_effect_t *)
{
	auto *wall = static_cast<display_wall_source *>(data);
	for (wall_child &child : wall->children) {
		if (!child.source)
			continue;

		const uint32_t source_w = obs_source_get_width(child.source);
		const uint32_t source_h = obs_source_get_height(child.source);
		const float scale_x =
			source_w > 0 ? static_cast<float>(child.width) /
					     static_cast<float>(source_w)
				     : 1.0f;
		const float scale_y =
			source_h > 0 ? static_cast<float>(child.height) /
					     static_cast<float>(source_h)
				     : 1.0f;

		gs_matrix_push();
		gs_matrix_translate3f(static_cast<float>(child.x),
				      static_cast<float>(child.y), 0.0f);
		if (scale_x != 1.0f || scale_y != 1.0f)
			gs_matrix_scale3f(scale_x, scale_y, 1.0f);
		obs_source_video_render(child.source);
		gs_matrix_pop();
	}
}

static void display_wall_tick(void *data, float seconds)
{
	auto *wall = static_cast<display_wall_source *>(data);
	// Before the refresh-interval early-out: the audio engine reconciles
	// captures and bearings on every tick.
	if (wall->audio)
		audio_wall_tick(wall->audio);
	if (wall->output_sync_timer_s >= 0.0f) {
		wall->output_sync_timer_s -= seconds;
		if (wall->output_sync_timer_s <= 0.0f)
			sync_obs_output_size(wall);
	}

	wall->refresh_timer_s += seconds;
	if (wall->refresh_timer_s < DISPLAY_REFRESH_INTERVAL_S)
		return;

	wall->refresh_timer_s = 0.0f;
	// Only run the expensive rebuild when the monitor topology actually
	// changed, or when a child capture source is missing and deserves a
	// retry. Settings changes rebuild directly via display_wall_update.
	bool missing_child = false;
	for (const wall_child &child : wall->children) {
		if (!child.source) {
			missing_child = true;
			break;
		}
	}
	if (!missing_child &&
	    wall->monitor_signature == monitor_topology_signature())
		return;
	rebuild_display_wall(wall);
}

static void display_wall_show(void *data)
{
	auto *wall = static_cast<display_wall_source *>(data);
	rebuild_display_wall(wall);
	add_active_children(wall);
	if (wall->audio)
		audio_wall_show(wall->audio);
}

static void display_wall_hide(void *data)
{
	auto *wall = static_cast<display_wall_source *>(data);
	remove_active_children(wall);
	if (wall->audio)
		audio_wall_hide(wall->audio);
}

static void display_wall_enum_active(void *data,
				     obs_source_enum_proc_t enum_callback,
				     void *param)
{
	auto *wall = static_cast<display_wall_source *>(data);
	for (wall_child &child : wall->children) {
		if (child.source)
			enum_callback(wall->context, child.source, param);
	}
	if (wall->audio)
		audio_wall_enum_active(wall->audio, enum_callback, param);
}

static bool display_wall_audio_render(void *, uint64_t *,
				      obs_source_audio_mix *, uint32_t, size_t,
				      size_t)
{
	return false;
}

static enum gs_color_space display_wall_get_color_space(
	void *, size_t, const enum gs_color_space *)
{
	return GS_CS_SRGB;
}

static obs_source_info display_wall_info = {};

} // namespace

size_t nyan_real_get_wall_monitor_map(nyan_wall_monitor_map *out,
				      size_t max_count)
{
	std::lock_guard<std::mutex> lk(g_wall_map_mutex);
	const size_t n = std::min(max_count, g_wall_map.size());
	for (size_t i = 0; i < n; i++)
		out[i] = g_wall_map[i];
	return n;
}

uint32_t nyan_real_wall_map_generation()
{
	return g_wall_map_gen.load(std::memory_order_relaxed);
}

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

namespace {

struct monitor_rect_lookup {
	const std::string *gdi_device = nullptr;
	RECT rect = {};
	bool found = false;
};

static BOOL CALLBACK monitor_rect_proc(HMONITOR handle, HDC, LPRECT rect,
				       LPARAM param)
{
	auto *ctx = reinterpret_cast<monitor_rect_lookup *>(param);
	MONITORINFOEXW mi = {};
	mi.cbSize = sizeof(mi);
	if (GetMonitorInfoW(handle, reinterpret_cast<LPMONITORINFO>(&mi)) &&
	    wide_to_utf8(mi.szDevice) == *ctx->gdi_device) {
		ctx->rect = *rect;
		ctx->found = true;
		return FALSE;
	}
	return TRUE;
}

} // namespace

bool nyan_real_find_glasses_display(nyan_real_glasses_display_info *out)
{
	for (const display_config_entry &entry :
	     get_active_display_config_entries()) {
		if (!is_glasses_display(entry.edid_vendor, entry.edid_product,
					entry.friendly_name))
			continue;
		if (out) {
			*out = {};
			out->gdi_device = entry.gdi_device;
			out->friendly_name = entry.friendly_name;
			monitor_rect_lookup lookup;
			lookup.gdi_device = &entry.gdi_device;
			EnumDisplayMonitors(nullptr, nullptr, monitor_rect_proc,
					    reinterpret_cast<LPARAM>(&lookup));
			if (lookup.found) {
				out->x = lookup.rect.left;
				out->y = lookup.rect.top;
				out->width = static_cast<uint32_t>(std::max<LONG>(
					0, lookup.rect.right - lookup.rect.left));
				out->height = static_cast<uint32_t>(std::max<LONG>(
					0, lookup.rect.bottom - lookup.rect.top));
				out->has_rect = true;
			}
		}
		return true;
	}
	return false;
}

void register_nyan_real_display_wall_source()
{
	display_wall_info.id = DISPLAY_WALL_ID;
	display_wall_info.type = OBS_SOURCE_TYPE_INPUT;
	display_wall_info.output_flags = OBS_SOURCE_VIDEO | OBS_SOURCE_CUSTOM_DRAW |
					 OBS_SOURCE_COMPOSITE | OBS_SOURCE_SRGB |
					 OBS_SOURCE_DO_NOT_DUPLICATE;
	display_wall_info.get_name = display_wall_get_name;
	display_wall_info.create = display_wall_create;
	display_wall_info.destroy = display_wall_destroy;
	display_wall_info.get_width = display_wall_get_width;
	display_wall_info.get_height = display_wall_get_height;
	display_wall_info.get_defaults = display_wall_defaults;
	display_wall_info.get_properties = display_wall_properties;
	display_wall_info.update = display_wall_update;
	display_wall_info.video_render = display_wall_render;
	display_wall_info.video_tick = display_wall_tick;
	display_wall_info.show = display_wall_show;
	display_wall_info.hide = display_wall_hide;
	display_wall_info.enum_active_sources = display_wall_enum_active;
	display_wall_info.enum_all_sources = display_wall_enum_active;
	display_wall_info.audio_render = display_wall_audio_render;
	display_wall_info.video_get_color_space = display_wall_get_color_space;
	display_wall_info.icon_type = OBS_ICON_TYPE_DESKTOP_CAPTURE;
	obs_register_source(&display_wall_info);
}
