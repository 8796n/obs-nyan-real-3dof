// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "monitor_enum.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "glasses_display.h"

namespace {

bool get_monitor_target_name(const wchar_t *device,
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

const display_config_entry *find_display_config_entry(
	const std::vector<display_config_entry> &entries, const std::string &gdi_device)
{
	for (const display_config_entry &entry : entries) {
		if (entry.gdi_device == gdi_device)
			return &entry;
	}
	return nullptr;
}

std::string get_friendly_monitor_name(HMONITOR handle)
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

std::string to_lower(std::string value)
{
	std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
		return static_cast<char>(std::tolower(ch));
	});
	return value;
}

std::string trim_copy(const std::string &value)
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

std::vector<std::string> split_filter_tokens(const std::string &text)
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

std::string searchable_text(const monitor_entry &monitor)
{
	std::string text = monitor.label + " " + monitor.id + " " + monitor.alt_id +
			   " " + monitor.friendly_name + " " + monitor.device_name;
	return to_lower(text);
}

bool parse_int_token(const std::string &token, int *value)
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

bool monitor_matches_token(const monitor_entry &monitor, const std::string &token)
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

BOOL CALLBACK enum_display_monitor_proc(HMONITOR handle, HDC, LPRECT rect,
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

BOOL CALLBACK monitor_signature_proc(HMONITOR handle, HDC, LPRECT rect,
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

const monitor_entry *find_monitor_by_token(
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

} // namespace

std::string wide_to_utf8(const wchar_t *value)
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

std::vector<display_config_entry> get_active_display_config_entries()
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

// Cheap monitor-topology snapshot for periodic change detection. The full
// enumerate_monitors() path resolves Windows display numbers and friendly
// names through QueryDisplayConfig and EnumDisplayDevices, which can block for
// milliseconds; running that every second inside video_tick stalls the video
// thread. This walks only EnumDisplayMonitors/GetMonitorInfo, so the periodic
// tick stays cheap until a display is added, removed, moved, or resized.
std::string monitor_topology_signature()
{
	std::ostringstream sig;
	EnumDisplayMonitors(nullptr, nullptr, monitor_signature_proc,
			    reinterpret_cast<LPARAM>(&sig));
	return sig.str();
}

std::vector<monitor_entry> enumerate_monitors()
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

std::vector<monitor_entry> filter_monitors(const std::vector<monitor_entry> &all,
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
		    nyan_real_is_glasses_display(monitor.edid_vendor, monitor.edid_product,
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

std::vector<std::vector<monitor_entry>>
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

std::vector<std::vector<monitor_entry>>
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

namespace {

struct monitor_rect_lookup {
	const std::string *gdi_device = nullptr;
	RECT rect = {};
	bool found = false;
};

BOOL CALLBACK monitor_rect_proc(HMONITOR handle, HDC, LPRECT rect, LPARAM param)
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
		if (!nyan_real_is_glasses_display(entry.edid_vendor, entry.edid_product,
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
