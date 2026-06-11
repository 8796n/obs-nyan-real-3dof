// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Audio Wall input source: the audio counterpart of the Display Wall. One
// source that discovers which apps are currently playing audio (WASAPI
// session enumeration), creates a private application-audio-capture child
// per app, derives each app's bearing from its window position on the
// desktop, and attaches a private spatial-audio filter to each child with
// that bearing. The children monitor directly (the filter's filter_add
// flips them to monitor-only), so the spatialized apps sum on the
// monitoring device. A composite source that mixes children itself was
// rejected: libobs forbids COMPOSITE|AUDIO, and composite audio never
// reaches the monitoring path (scenes cannot be monitored either) - the
// glasses listen over monitoring, so per-child monitoring is the channel
// that actually works.
#include <obs-module.h>
#include <media-io/audio-io.h>
#include <util/platform.h>

#include <windows.h>
#include <mmdeviceapi.h>
#include <audiopolicy.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "audio-wall-source.h"
#include "display-wall-source.h"
#include "spatial_pan.h"
#include "tooltip_util.h"
#include "ws_audio_server.h"

namespace {

constexpr const char *APP_AUDIO_CAPTURE_ID = "wasapi_process_output_capture";
// Push-audio sink for browser-extension streams (registered below,
// CAP_DISABLED keeps it out of the add-source menu).
constexpr const char *WS_AUDIO_SOURCE_ID = "nyan_real_3dof_ws_audio";
constexpr uint64_t WS_STREAM_TIMEOUT_NS = 5000000000ULL;
// Version of the extension ingest protocol ("v" in meta messages). The
// extension auto-updates from the store while the plugin updates manually,
// so mismatches must be loud instead of silently broken. Bump together with
// PROTOCOL_VERSION in tools/chrome-extension/background.js on breaking
// changes only (additive fields don't count); see CONTRIBUTING.md.
constexpr long long WS_PROTOCOL_VERSION = 1;
// win-wasapi window-helpers: WINDOW_PRIORITY_EXE. Matching by exe keeps the
// capture attached while window titles change (browser tabs).
constexpr int WINDOW_PRIORITY_EXE = 2;
constexpr auto POLL_INTERVAL = std::chrono::milliseconds(2000);

// OBS itself (its monitoring output is an active session on the glasses'
// endpoint) must never be captured: instant feedback loop.
const char *BUILTIN_EXCLUDES[] = {"obs64.exe", "obs32.exe", "obs.exe"};

std::string to_lower(std::string s)
{
	std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
		return static_cast<char>(std::tolower(c));
	});
	return s;
}

// win-wasapi's window string fields escape '#' and ':' (see its
// window-helpers encode_dstr).
std::string encode_window_field(const std::string &s)
{
	std::string out;
	for (char c : s) {
		if (c == '#')
			out += "#22";
		else if (c == ':')
			out += "#3A";
		else
			out += c;
	}
	return out;
}

std::string wide_to_utf8(const wchar_t *w)
{
	char *p = nullptr;
	os_wcs_to_utf8_ptr(w, 0, &p);
	std::string out = p ? p : "";
	bfree(p);
	return out;
}

// Executable base name (lowercase) of a process, "" when unavailable.
std::string pid_exe_lower(DWORD pid)
{
	if (!pid)
		return "";
	HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
	if (!h)
		return "";
	wchar_t path[MAX_PATH];
	DWORD len = MAX_PATH;
	std::string out;
	if (QueryFullProcessImageNameW(h, 0, path, &len)) {
		const wchar_t *base = wcsrchr(path, L'\\');
		out = to_lower(wide_to_utf8(base ? base + 1 : path));
	}
	CloseHandle(h);
	return out;
}

struct window_info {
	HWND hwnd = nullptr;
	DWORD pid = 0;
	RECT rect = {};
	long long area = 0;
};

BOOL CALLBACK collect_windows_cb(HWND hwnd, LPARAM param)
{
	auto *windows = reinterpret_cast<std::vector<window_info> *>(param);
	if (!IsWindowVisible(hwnd))
		return TRUE;
	const LONG_PTR ex = GetWindowLongPtrW(hwnd, GWL_EXSTYLE);
	if (ex & WS_EX_TOOLWINDOW)
		return TRUE;
	if (GetWindowTextLengthW(hwnd) == 0)
		return TRUE;

	window_info info;
	info.hwnd = hwnd;
	GetWindowThreadProcessId(hwnd, &info.pid);
	if (!info.pid)
		return TRUE;
	// Minimized windows park at (-32000, -32000); use the restored rect
	// so the app keeps its on-screen position while iconified.
	if (IsIconic(hwnd)) {
		WINDOWPLACEMENT wp = {};
		wp.length = sizeof(wp);
		if (!GetWindowPlacement(hwnd, &wp))
			return TRUE;
		info.rect = wp.rcNormalPosition;
	} else if (!GetWindowRect(hwnd, &info.rect)) {
		return TRUE;
	}
	info.area = static_cast<long long>(info.rect.right - info.rect.left) *
		    static_cast<long long>(info.rect.bottom - info.rect.top);
	if (info.area <= 0)
		return TRUE;
	windows->push_back(info);
	return TRUE;
}

// PIDs with an active (currently rendering) audio session on any output
// device. System-sounds sessions are skipped. Apps moved off the default
// device still show up because every active endpoint is walked.
std::vector<DWORD> active_audio_pids()
{
	std::vector<DWORD> pids;
	IMMDeviceEnumerator *devenum = nullptr;
	if (FAILED(CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr,
				    CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
				    reinterpret_cast<void **>(&devenum))))
		return pids;
	IMMDeviceCollection *devices = nullptr;
	if (SUCCEEDED(devenum->EnumAudioEndpoints(eRender, DEVICE_STATE_ACTIVE,
						  &devices))) {
		UINT dev_count = 0;
		devices->GetCount(&dev_count);
		for (UINT i = 0; i < dev_count; i++) {
			IMMDevice *dev = nullptr;
			if (FAILED(devices->Item(i, &dev)))
				continue;
			IAudioSessionManager2 *mgr = nullptr;
			if (SUCCEEDED(dev->Activate(
				    __uuidof(IAudioSessionManager2), CLSCTX_ALL,
				    nullptr,
				    reinterpret_cast<void **>(&mgr)))) {
				IAudioSessionEnumerator *sessions = nullptr;
				if (SUCCEEDED(mgr->GetSessionEnumerator(
					    &sessions))) {
					int count = 0;
					sessions->GetCount(&count);
					for (int j = 0; j < count; j++) {
						IAudioSessionControl *ctl =
							nullptr;
						if (FAILED(sessions->GetSession(
							    j, &ctl)))
							continue;
						AudioSessionState state =
							AudioSessionStateInactive;
						ctl->GetState(&state);
						IAudioSessionControl2 *ctl2 =
							nullptr;
						if (state == AudioSessionStateActive &&
						    SUCCEEDED(ctl->QueryInterface(
							    __uuidof(IAudioSessionControl2),
							    reinterpret_cast<void **>(
								    &ctl2)))) {
							DWORD pid = 0;
							if (ctl2->IsSystemSoundsSession() !=
								    S_OK &&
							    SUCCEEDED(ctl2->GetProcessId(
								    &pid)) &&
							    pid)
								pids.push_back(
									pid);
						}
						if (ctl2)
							ctl2->Release();
						ctl->Release();
					}
					sessions->Release();
				}
				mgr->Release();
			}
			dev->Release();
		}
		devices->Release();
	}
	devenum->Release();
	return pids;
}

// ---- source ----------------------------------------------------------------

struct desired_app {
	std::string exe;    // lowercase basename, identity key
	std::string window; // encoded "title:class:exe" for win-wasapi
	float norm_x;       // window center across the virtual desktop, -0.5..0.5
};

struct audio_wall_child {
	obs_source_t *source = nullptr; // private app audio capture
	obs_source_t *filter = nullptr; // private spatial filter on it
	std::string exe;
	float norm_x = 0.0f;
};

// A PCM stream pushed by the browser extension over the WebSocket ingest.
// The extension owns the geometry (it reports norm_x directly) and the tab
// audio is detached from the OS output by the page-side Web Audio hook, so
// no app capture and no "move the default device" workaround is involved.
struct ws_stream {
	obs_source_t *source = nullptr; // private push-audio sink
	obs_source_t *filter = nullptr; // private spatial filter on it
	std::string label;
	std::string exe; // reporting browser, excluded from app discovery
	float norm_x = 0.0f;
	uint32_t sample_rate = 48000;
	int channels = 2;
	uint64_t last_rx_ns = 0;
};

struct audio_wall_source {
	obs_source_t *context = nullptr;
	std::atomic<int> mode{SPATIAL_MODE_POINT};
	std::atomic<bool> distance_gain{true};
	// Geometry snapshot for change detection, tick thread only.
	float geo_half_w = -1.0f;
	float geo_dist = -1.0f;
	float geo_curve = -1.0f;
	uint32_t geo_map_gen = 0;

	std::mutex exclude_mutex;
	std::vector<std::string> exclude; // lowercase exe names

	// Children are reconciled on the tick thread, fed from WS connection
	// threads, and enumerated from UI threads; one mutex guards them all
	// (every holder is short).
	std::mutex children_mutex;
	std::vector<std::unique_ptr<audio_wall_child>> children;
	std::map<uint64_t, ws_stream> ws_streams; // key: conn<<32 | stream id
	std::set<uint64_t> ws_warned_conns; // protocol-mismatch warnings sent
	// Last mismatched extension protocol version, -1 = none; shown in the
	// source properties so the user sees it without opening the log.
	std::atomic<long long> ws_proto_mismatch{-1};
	// The capture list shown in the properties changed; tick tells OBS to
	// rebuild an open properties view (no-op while it is closed).
	std::atomic<bool> props_dirty{false};
	bool active_children = false;

	// Browser exes currently streaming over WS; the poll thread excludes
	// them from app discovery so the same audio is not captured twice.
	std::mutex ws_exe_mutex;
	std::set<std::string> ws_exes;

	ws_audio_server ws_server;

	// Poll thread output; tick applies it when the generation moves.
	std::mutex desired_mutex;
	std::vector<desired_app> desired;
	uint64_t desired_gen = 0;
	uint64_t applied_gen = 0;
	// Set by update(): push spread/mode/distance into the child filters.
	std::atomic<bool> filters_dirty{false};

	std::thread poll_thread;
	std::mutex poll_mutex;
	std::condition_variable poll_cv;
	bool stop = false;
	// Discovery summary last logged, poll thread only.
	std::string last_discovery_log;
};

// Wall texture coordinate of a physical desktop x position, using the
// Display Wall's published layout. Positions outside the wall monitors
// (excluded displays, the glasses display...) extrapolate linearly with the
// nearest monitor's scale, so a screen to the right of the wall sounds from
// beyond the wall's right edge instead of merging with it; the result may
// leave 0..1 and the caller bounds it. False when no wall exists.
bool wall_u_from_desktop_x(double x, double *u_out)
{
	nyan_wall_monitor_map map[16];
	const size_t n = nyan_real_get_wall_monitor_map(map, 16);
	if (!n)
		return false;
	const nyan_wall_monitor_map *nearest = nullptr;
	double nearest_dist = 0.0;
	for (size_t i = 0; i < n; i++) {
		const nyan_wall_monitor_map &m = map[i];
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

// Exact bearing of a desktop position as rendered by the virtual screen:
// desktop x -> wall texture u -> world position on the (flat or curved)
// screen -> azimuth from the viewer. False only before the first virtual
// screen render of the session (the published geometry is sticky after
// that); the caller then uses a plain 60-degree linear stage.
bool auto_azimuth_deg(float norm_x, double *out_deg)
{
	const double half_w =
		g_device.screen_half_width_m.load(std::memory_order_relaxed);
	const double dist = clampd(
		g_device.screen_distance_m.load(std::memory_order_relaxed),
		MIN_SCREEN_DISTANCE_M, MAX_SCREEN_DISTANCE_M);
	if (half_w <= 1e-4)
		return false;

	const int vx = GetSystemMetrics(SM_XVIRTUALSCREEN);
	const int vw = std::max(1, GetSystemMetrics(SM_CXVIRTUALSCREEN));
	const double desk_x =
		vx + (clampd(norm_x, -0.5, 0.5) + 0.5) * vw;
	double u;
	if (!wall_u_from_desktop_x(desk_x, &u))
		u = clampd(norm_x, -0.5, 0.5) + 0.5; // wall == desktop
	// Up to half a wall width beyond each edge for off-wall displays;
	// the final bearing is clamped to +-90 by the filter anyway.
	const double off = (clampd(u, -0.5, 1.5) - 0.5) * 2.0 * half_w;

	const double curve = clampd(
		g_device.screen_curve.load(std::memory_order_relaxed), 0.0,
		MAX_SCREEN_CURVE);
	double az;
	if (curve <= 0.0001) {
		az = std::atan2(off, dist);
	} else {
		// Same cylinder as the warp shader: radius D/curve, screen
		// center kept at distance D, off is the arc length.
		const double radius = dist / curve;
		const double theta = off / radius;
		az = std::atan2(radius * std::sin(theta),
				dist - radius * (1.0 - std::cos(theta)));
	}
	*out_deg = az * 180.0 / PI;
	return true;
}

// Push the wall settings and a child's bearing into its spatial filter.
// The filter clamps its bearing UI to +-90, matching spread/2's maximum.
void update_spatial_filter(audio_wall_source *wall, obs_source_t *filter,
			   float norm_x)
{
	if (!filter)
		return;
	double az_deg;
	if (!auto_azimuth_deg(norm_x, &az_deg))
		az_deg = norm_x * 60.0; // cold start, before any render
	obs_data_t *settings = obs_data_create();
	obs_data_set_double(settings, "azimuth_deg",
			    clampd(az_deg, -90.0, 90.0));
	obs_data_set_int(settings, "mode",
			 wall->mode.load(std::memory_order_relaxed));
	obs_data_set_bool(settings, "distance_gain",
			  wall->distance_gain.load(std::memory_order_relaxed));
	obs_source_update(filter, settings);
	obs_data_release(settings);
}

void update_child_filter(audio_wall_source *wall, audio_wall_child *child)
{
	update_spatial_filter(wall, child->filter, child->norm_x);
}

bool is_excluded(audio_wall_source *wall, const std::string &exe)
{
	for (const char *b : BUILTIN_EXCLUDES) {
		if (exe == b)
			return true;
	}
	{
		std::lock_guard<std::mutex> lk(wall->ws_exe_mutex);
		if (wall->ws_exes.count(exe))
			return true;
	}
	std::lock_guard<std::mutex> lk(wall->exclude_mutex);
	return std::find(wall->exclude.begin(), wall->exclude.end(), exe) !=
	       wall->exclude.end();
}

BOOL CALLBACK collect_child_pids_cb(HWND hwnd, LPARAM param)
{
	auto *pids = reinterpret_cast<std::set<DWORD> *>(param);
	DWORD pid = 0;
	GetWindowThreadProcessId(hwnd, &pid);
	if (pid)
		pids->insert(pid);
	return TRUE;
}

// One discovery pass: who plays audio, where their window sits.
std::vector<desired_app> discover_apps(audio_wall_source *wall)
{
	std::vector<desired_app> out;
	std::vector<std::string> skipped;

	std::set<std::string> exes;
	for (DWORD pid : active_audio_pids()) {
		const std::string exe = pid_exe_lower(pid);
		if (!exe.empty() && !is_excluded(wall, exe))
			exes.insert(exe);
	}

	std::vector<window_info> windows;
	std::map<DWORD, std::string> window_pid_exe;
	if (!exes.empty()) {
		EnumWindows(collect_windows_cb,
			    reinterpret_cast<LPARAM>(&windows));
		for (const window_info &w : windows) {
			if (window_pid_exe.find(w.pid) ==
			    window_pid_exe.end())
				window_pid_exe[w.pid] = pid_exe_lower(w.pid);
		}
	}

	// Child-window exe sets of ApplicationFrameHost frames, resolved
	// lazily: UWP/store apps (Media Player, Films & TV...) own only a
	// child window inside a frame-host window, so an exe-based top-level
	// search never finds them.
	std::map<HWND, std::set<std::string>> frame_child_exes;
	auto frame_children = [&](const window_info &frame)
		-> const std::set<std::string> & {
		auto it = frame_child_exes.find(frame.hwnd);
		if (it == frame_child_exes.end()) {
			std::set<DWORD> pids;
			EnumChildWindows(frame.hwnd, collect_child_pids_cb,
					 reinterpret_cast<LPARAM>(&pids));
			std::set<std::string> child_exes;
			for (DWORD pid : pids) {
				const std::string e = pid_exe_lower(pid);
				if (!e.empty())
					child_exes.insert(e);
			}
			it = frame_child_exes
				     .emplace(frame.hwnd,
					      std::move(child_exes))
				     .first;
		}
		return it->second;
	};

	const int vx = GetSystemMetrics(SM_XVIRTUALSCREEN);
	const int vw = std::max(1, GetSystemMetrics(SM_CXVIRTUALSCREEN));

	for (const std::string &exe : exes) {
		// The audio session often lives in a windowless child process
		// (browsers), so the window is matched by exe name across all
		// processes; the largest window wins.
		const window_info *best = nullptr;
		for (const window_info &w : windows) {
			if (window_pid_exe[w.pid] != exe)
				continue;
			if (!best || w.area > best->area)
				best = &w;
		}
		if (!best) {
			// UWP fallback: pick the largest frame-host window
			// that hosts a child window of this exe. The window
			// string still carries the app's exe; win-wasapi's
			// matching resolves frame-hosted apps the same way.
			for (const window_info &w : windows) {
				if (window_pid_exe[w.pid] !=
				    "applicationframehost.exe")
					continue;
				if (!frame_children(w).count(exe))
					continue;
				if (!best || w.area > best->area)
					best = &w;
			}
		}
		if (!best) {
			skipped.push_back(exe);
			continue; // background audio without a window
		}

		const double cx = 0.5 * (best->rect.left + best->rect.right);
		const double nx =
			clampd((cx - vx) / static_cast<double>(vw), 0.0, 1.0) -
			0.5;

		wchar_t title[256] = {};
		wchar_t cls[256] = {};
		GetWindowTextW(best->hwnd, title, 255);
		GetClassNameW(best->hwnd, cls, 255);
		desired_app app;
		app.exe = exe;
		app.window = encode_window_field(wide_to_utf8(title)) + ":" +
			     encode_window_field(wide_to_utf8(cls)) + ":" +
			     encode_window_field(exe);
		app.norm_x = static_cast<float>(nx);
		out.push_back(std::move(app));
	}
	std::sort(out.begin(), out.end(),
		  [](const desired_app &a, const desired_app &b) {
			  return a.exe < b.exe;
		  });

	// Discovery summary on change: makes "app played audio but was
	// skipped" visible instead of silent (UWP quirks, tray players...).
	std::string summary = "capturing:";
	for (const desired_app &a : out)
		summary += " " + a.exe;
	if (!skipped.empty()) {
		summary += " / no window:";
		for (const std::string &e : skipped)
			summary += " " + e;
	}
	if (summary != wall->last_discovery_log) {
		wall->last_discovery_log = summary;
		blog(LOG_INFO, "[obs-nyan-real-3dof] audio wall: %s",
		     summary.c_str());
	}
	return out;
}

void poll_thread_fn(audio_wall_source *wall)
{
	const HRESULT com = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
	std::vector<desired_app> last;
	bool first = true;
	for (;;) {
		{
			std::unique_lock<std::mutex> lk(wall->poll_mutex);
			if (wall->stop)
				break;
			wall->poll_cv.wait_for(lk, POLL_INTERVAL);
			if (wall->stop)
				break;
		}
		std::vector<desired_app> apps = discover_apps(wall);
		bool changed = first || apps.size() != last.size();
		for (size_t i = 0; !changed && i < apps.size(); i++) {
			changed = apps[i].exe != last[i].exe ||
				  std::fabs(apps[i].norm_x - last[i].norm_x) >
					  0.01f;
		}
		if (changed) {
			last = apps;
			first = false;
			std::lock_guard<std::mutex> lk(wall->desired_mutex);
			wall->desired = std::move(apps);
			wall->desired_gen++;
		}
	}
	if (SUCCEEDED(com))
		CoUninitialize();
}

void wall_remove_active_children(audio_wall_source *wall)
{
	if (!wall->active_children)
		return;
	for (auto &child : wall->children) {
		if (child->source)
			obs_source_remove_active_child(wall->context,
						       child->source);
	}
	for (auto &entry : wall->ws_streams) {
		if (entry.second.source)
			obs_source_remove_active_child(wall->context,
						       entry.second.source);
	}
	wall->active_children = false;
}

void wall_add_active_children(audio_wall_source *wall)
{
	if (wall->active_children || !obs_source_showing(wall->context))
		return;
	for (auto &child : wall->children) {
		if (child->source)
			obs_source_add_active_child(wall->context,
						    child->source);
	}
	for (auto &entry : wall->ws_streams) {
		if (entry.second.source)
			obs_source_add_active_child(wall->context,
						    entry.second.source);
	}
	wall->active_children = true;
}

// ---- WebSocket stream handling (runs on the server's conn threads) ---------

// Caller holds children_mutex.
void rebuild_ws_exes_locked(audio_wall_source *wall)
{
	std::set<std::string> exes;
	for (auto &entry : wall->ws_streams) {
		if (!entry.second.exe.empty())
			exes.insert(entry.second.exe);
	}
	std::lock_guard<std::mutex> lk(wall->ws_exe_mutex);
	wall->ws_exes = std::move(exes);
}

// Caller holds children_mutex.
void release_ws_stream_locked(audio_wall_source *wall, ws_stream &st)
{
	if (st.source) {
		if (wall->active_children)
			obs_source_remove_active_child(wall->context,
						       st.source);
		obs_source_release(st.source);
	}
	if (st.filter)
		obs_source_release(st.filter);
}

void wall_ws_text(audio_wall_source *wall, uint64_t conn, obs_data_t *msg)
{
	const char *type = obs_data_get_string(msg, "type");
	const uint64_t key = (conn << 32) |
			     (static_cast<uint64_t>(obs_data_get_int(
				      msg, "stream")) &
			      0xFFFFFFFFULL);
	if (type && strcmp(type, "close") == 0) {
		std::lock_guard<std::mutex> lk(wall->children_mutex);
		auto it = wall->ws_streams.find(key);
		if (it != wall->ws_streams.end()) {
			blog(LOG_INFO,
			     "[obs-nyan-real-3dof] audio wall: ws stream closed '%s'",
			     it->second.label.c_str());
			release_ws_stream_locked(wall, it->second);
			wall->ws_streams.erase(it);
			rebuild_ws_exes_locked(wall);
			wall->props_dirty.store(true,
						std::memory_order_relaxed);
		}
		return;
	}
	if (!type || strcmp(type, "meta") != 0)
		return;

	std::lock_guard<std::mutex> lk(wall->children_mutex);
	const long long proto = obs_data_get_int(msg, "v");
	if (proto != WS_PROTOCOL_VERSION) {
		if (wall->ws_proto_mismatch.exchange(
			    proto, std::memory_order_relaxed) != proto)
			wall->props_dirty.store(true, std::memory_order_relaxed);
		if (!wall->ws_warned_conns.count(conn)) {
			wall->ws_warned_conns.insert(conn);
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] audio wall: extension speaks ingest protocol v%lld but this plugin expects v%lld; update the older side (extension: tools/chrome-extension)",
			     proto, WS_PROTOCOL_VERSION);
		}
	} else {
		if (wall->ws_proto_mismatch.exchange(
			    -1, std::memory_order_relaxed) != -1)
			wall->props_dirty.store(true, std::memory_order_relaxed);
	}
	ws_stream &st = wall->ws_streams[key];
	st.norm_x = static_cast<float>(
		clampd(obs_data_get_double(msg, "norm_x"), -0.5, 0.5));
	const char *label = obs_data_get_string(msg, "label");
	if (label && *label && st.label != label) {
		st.label = label;
		wall->props_dirty.store(true, std::memory_order_relaxed);
	}
	const long long sr = obs_data_get_int(msg, "sample_rate");
	if (sr >= 8000 && sr <= 192000)
		st.sample_rate = static_cast<uint32_t>(sr);
	st.channels = obs_data_get_int(msg, "channels") == 1 ? 1 : 2;
	const char *exe = obs_data_get_string(msg, "exe");
	if (exe && *exe)
		st.exe = to_lower(exe);
	st.last_rx_ns = os_gettime_ns();

	if (!st.source) {
		char name[64];
		snprintf(name, sizeof(name),
			 "nyan Real Audio Wall - WS %llu",
			 static_cast<unsigned long long>(key));
		st.source = obs_source_create_private(WS_AUDIO_SOURCE_ID, name,
						      nullptr);
		if (st.source) {
			// Unbuffered playback for lip sync. This was a
			// crackle source while the extension captured on the
			// page's main thread (dropouts), but the AudioWorklet
			// capture delivers steady ~10.7 ms chunks over a
			// TCP_NODELAY socket; revert to buffered if pops
			// return on loaded systems.
			obs_source_set_async_unbuffered(st.source, true);
			st.filter = obs_source_create_private(
				"nyan_real_3dof_spatial_audio",
				(std::string(name) + " (spatial)").c_str(),
				nullptr);
			if (st.filter)
				obs_source_filter_add(st.source, st.filter);
			if (wall->active_children)
				obs_source_add_active_child(wall->context,
							    st.source);
			blog(LOG_INFO,
			     "[obs-nyan-real-3dof] audio wall: ws stream '%s' (norm x %.2f, %u Hz, %dch)",
			     st.label.c_str(), st.norm_x, st.sample_rate,
			     st.channels);
			wall->props_dirty.store(true,
						std::memory_order_relaxed);
		}
	}
	update_spatial_filter(wall, st.filter, st.norm_x);
	rebuild_ws_exes_locked(wall);
}

void wall_ws_binary(audio_wall_source *wall, uint64_t conn, const uint8_t *data,
		    size_t len)
{
	if (len < 4)
		return;
	const uint32_t sid = static_cast<uint32_t>(data[0]) |
			     (static_cast<uint32_t>(data[1]) << 8) |
			     (static_cast<uint32_t>(data[2]) << 16) |
			     (static_cast<uint32_t>(data[3]) << 24);
	const uint64_t key = (conn << 32) | sid;
	obs_source_t *source = nullptr;
	uint32_t sample_rate = 48000;
	int channels = 2;
	{
		std::lock_guard<std::mutex> lk(wall->children_mutex);
		auto it = wall->ws_streams.find(key);
		if (it == wall->ws_streams.end() || !it->second.source)
			return;
		it->second.last_rx_ns = os_gettime_ns();
		source = obs_source_get_ref(it->second.source);
		sample_rate = it->second.sample_rate;
		channels = it->second.channels;
	}
	if (!source)
		return;
	const uint32_t frames = static_cast<uint32_t>(
		(len - 4) / (sizeof(int16_t) * channels));
	if (frames) {
		obs_source_audio audio = {};
		audio.data[0] = data + 4;
		audio.frames = frames;
		audio.speakers = channels == 1 ? SPEAKERS_MONO
					       : SPEAKERS_STEREO;
		audio.format = AUDIO_FORMAT_16BIT;
		audio.samples_per_sec = sample_rate;
		audio.timestamp = os_gettime_ns();
		obs_source_output_audio(source, &audio);
	}
	obs_source_release(source);
}

void wall_ws_closed(audio_wall_source *wall, uint64_t conn)
{
	std::lock_guard<std::mutex> lk(wall->children_mutex);
	for (auto it = wall->ws_streams.begin();
	     it != wall->ws_streams.end();) {
		if ((it->first >> 32) == conn) {
			release_ws_stream_locked(wall, it->second);
			it = wall->ws_streams.erase(it);
			wall->props_dirty.store(true,
						std::memory_order_relaxed);
		} else {
			++it;
		}
	}
	wall->ws_warned_conns.erase(conn);
	rebuild_ws_exes_locked(wall);
}

void start_ws_server(audio_wall_source *wall, uint16_t port)
{
	ws_server_callbacks cb;
	cb.on_text = [wall](uint64_t conn, obs_data_t *msg) {
		wall_ws_text(wall, conn, msg);
	};
	cb.on_binary = [wall](uint64_t conn, const uint8_t *data, size_t len) {
		wall_ws_binary(wall, conn, data, len);
	};
	cb.on_close = [wall](uint64_t conn) { wall_ws_closed(wall, conn); };
	wall->ws_server.start(port, std::move(cb));
}

// Apply the poll thread's desired app list: create captures for new apps,
// update positions, drop captures of apps that stopped playing.
void wall_reconcile(audio_wall_source *wall)
{
	std::vector<desired_app> want;
	{
		std::lock_guard<std::mutex> lk(wall->desired_mutex);
		if (wall->applied_gen == wall->desired_gen)
			return;
		want = wall->desired;
		wall->applied_gen = wall->desired_gen;
	}

	std::lock_guard<std::mutex> lk(wall->children_mutex);
	for (auto it = wall->children.begin(); it != wall->children.end();) {
		const bool keep =
			std::any_of(want.begin(), want.end(),
				    [&](const desired_app &a) {
					    return a.exe == (*it)->exe;
				    });
		if (keep) {
			++it;
			continue;
		}
		if ((*it)->source) {
			if (wall->active_children)
				obs_source_remove_active_child(wall->context,
							       (*it)->source);
			obs_source_release((*it)->source);
		}
		if ((*it)->filter)
			obs_source_release((*it)->filter);
		blog(LOG_INFO, "[obs-nyan-real-3dof] audio wall: released '%s'",
		     (*it)->exe.c_str());
		it = wall->children.erase(it);
		wall->props_dirty.store(true, std::memory_order_relaxed);
	}

	for (const desired_app &app : want) {
		auto found = std::find_if(
			wall->children.begin(), wall->children.end(),
			[&](const std::unique_ptr<audio_wall_child> &c) {
				return c->exe == app.exe;
			});
		if (found != wall->children.end()) {
			if (std::fabs((*found)->norm_x - app.norm_x) > 0.001f) {
				(*found)->norm_x = app.norm_x;
				update_child_filter(wall, found->get());
			}
			continue;
		}
		obs_data_t *settings = obs_data_create();
		obs_data_set_string(settings, "window", app.window.c_str());
		obs_data_set_int(settings, "priority", WINDOW_PRIORITY_EXE);
		const std::string name = "nyan Real Audio Wall - " + app.exe;
		obs_source_t *source = obs_source_create_private(
			APP_AUDIO_CAPTURE_ID, name.c_str(), settings);
		obs_data_release(settings);
		if (!source) {
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] audio wall: capture create failed for '%s'",
			     app.exe.c_str());
			continue;
		}
		auto child = std::make_unique<audio_wall_child>();
		child->source = source;
		child->exe = app.exe;
		child->norm_x = app.norm_x;
		// The spatial filter does the head-tracked panning, and its
		// filter_add callback flips the capture to monitor-only, so
		// the child is audible on the monitoring device right away.
		child->filter = obs_source_create_private(
			"nyan_real_3dof_spatial_audio",
			(name + " (spatial)").c_str(), nullptr);
		if (child->filter) {
			obs_source_filter_add(source, child->filter);
			update_child_filter(wall, child.get());
		} else {
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] audio wall: spatial filter create failed for '%s'",
			     app.exe.c_str());
		}
		if (wall->active_children)
			obs_source_add_active_child(wall->context, source);
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] audio wall: capturing '%s' (norm x %.2f)",
		     app.exe.c_str(), app.norm_x);
		wall->children.push_back(std::move(child));
		wall->props_dirty.store(true, std::memory_order_relaxed);
	}
}

const char *wall_get_name(void *)
{
	return obs_module_text("audiowall.name");
}

void wall_update(void *data, obs_data_t *settings)
{
	auto *wall = static_cast<audio_wall_source *>(data);
	wall->mode.store(static_cast<int>(obs_data_get_int(settings, "mode")),
			 std::memory_order_relaxed);
	wall->distance_gain.store(obs_data_get_bool(settings, "distance_gain"),
				  std::memory_order_relaxed);

	std::vector<std::string> exclude;
	std::string item;
	const char *raw = obs_data_get_string(settings, "exclude");
	for (const char *p = raw ? raw : "";; p++) {
		if (*p && *p != ',' && *p != ';') {
			if (!std::isspace(static_cast<unsigned char>(*p)))
				item += *p;
			continue;
		}
		if (!item.empty()) {
			exclude.push_back(to_lower(item));
			item.clear();
		}
		if (!*p)
			break;
	}
	{
		std::lock_guard<std::mutex> lk(wall->exclude_mutex);
		wall->exclude = std::move(exclude);
	}
	wall->filters_dirty.store(true, std::memory_order_relaxed);

	// Browser-extension ingest port. A failed bind (port in use) is
	// retried on the next settings change.
	long long port_setting = obs_data_get_int(settings, "ws_port");
	if (port_setting < 1024 || port_setting > 65535)
		port_setting = 8796;
	const uint16_t port = static_cast<uint16_t>(port_setting);
	if (!wall->ws_server.running() || wall->ws_server.port() != port)
		start_ws_server(wall, port);
}

void *wall_create(obs_data_t *settings, obs_source_t *context)
{
	auto *wall = new audio_wall_source();
	wall->context = context;
	wall_update(wall, settings);
	wall->poll_thread = std::thread(poll_thread_fn, wall);
	blog(LOG_INFO, "[obs-nyan-real-3dof] audio wall source created");
	return wall;
}

void wall_destroy(void *data)
{
	auto *wall = static_cast<audio_wall_source *>(data);
	// Join the WS connection threads before touching the children they
	// feed; after stop() no callback can run.
	wall->ws_server.stop();
	{
		std::lock_guard<std::mutex> lk(wall->poll_mutex);
		wall->stop = true;
	}
	wall->poll_cv.notify_all();
	if (wall->poll_thread.joinable())
		wall->poll_thread.join();
	{
		std::lock_guard<std::mutex> lk(wall->children_mutex);
		wall_remove_active_children(wall);
		for (auto &child : wall->children) {
			if (child->source)
				obs_source_release(child->source);
			if (child->filter)
				obs_source_release(child->filter);
		}
		wall->children.clear();
		for (auto &entry : wall->ws_streams)
			release_ws_stream_locked(wall, entry.second);
		wall->ws_streams.clear();
	}
	delete wall;
}

void wall_defaults(obs_data_t *settings)
{
	obs_data_set_default_int(settings, "mode", SPATIAL_MODE_POINT);
	obs_data_set_default_bool(settings, "distance_gain", true);
	obs_data_set_default_string(settings, "exclude", "");
	obs_data_set_default_int(settings, "ws_port", 8796);
}

// ---- push-audio sink for WS streams ----------------------------------------
// A do-nothing async audio source; the WS handler pushes PCM into it with
// obs_source_output_audio. CAP_DISABLED keeps it out of the add-source menu.

const char *ws_audio_get_name(void *)
{
	return obs_module_text("wsaudio.name");
}

void *ws_audio_create(obs_data_t *, obs_source_t *)
{
	return bzalloc(1);
}

void ws_audio_destroy(void *data)
{
	bfree(data);
}

obs_properties_t *wall_properties(void *data)
{
	auto *wall = static_cast<audio_wall_source *>(data);
	obs_properties_t *props = obs_properties_create();
	obs_properties_add_text(props, "audiowall_notice",
				obs_module_text("audiowall.notice"),
				OBS_TEXT_INFO);

	std::string current = obs_module_text("audiowall.current");
	if (wall) {
		const long long mismatch =
			wall->ws_proto_mismatch.load(std::memory_order_relaxed);
		if (mismatch >= 0) {
			char warn[160];
			snprintf(warn, sizeof(warn),
				 obs_module_text("audiowall.proto_mismatch"),
				 mismatch,
				 static_cast<long long>(WS_PROTOCOL_VERSION));
			current = std::string(warn) + "\n" + current;
		}
		std::lock_guard<std::mutex> lk(wall->children_mutex);
		if (wall->children.empty() && wall->ws_streams.empty()) {
			current += " ";
			current += obs_module_text("audiowall.current_none");
		} else {
			for (auto &child : wall->children) {
				current += "\n  ";
				current += child->exe;
			}
			for (auto &entry : wall->ws_streams) {
				current += "\n  [WS] ";
				current += entry.second.label;
			}
		}
	}
	obs_properties_add_text(props, "audiowall_current", current.c_str(),
				OBS_TEXT_INFO);

	obs_property_t *mode = obs_properties_add_list(
		props, "mode", obs_module_text("spatial.mode"),
		OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
	obs_property_list_add_int(mode, obs_module_text("spatial.mode.point"),
				  SPATIAL_MODE_POINT);
	obs_property_list_add_int(mode, obs_module_text("spatial.mode.balance"),
				  SPATIAL_MODE_BALANCE);
	obs_property_t *dist = obs_properties_add_bool(
		props, "distance_gain",
		obs_module_text("spatial.distance_gain"));
	obs_property_set_long_description(
		dist, wrapped_tooltip("spatial.distance_gain_tooltip").c_str());
	obs_property_t *exclude = obs_properties_add_text(
		props, "exclude", obs_module_text("audiowall.exclude"),
		OBS_TEXT_DEFAULT);
	obs_property_set_long_description(
		exclude, wrapped_tooltip("audiowall.exclude_tooltip").c_str());
	obs_property_t *port = obs_properties_add_int(
		props, "ws_port", obs_module_text("audiowall.ws_port"), 1024,
		65535, 1);
	obs_property_set_long_description(
		port, wrapped_tooltip("audiowall.ws_port_tooltip").c_str());
	return props;
}

void wall_tick(void *data, float)
{
	auto *wall = static_cast<audio_wall_source *>(data);
	wall_reconcile(wall);
	// Refresh an open properties dialog when the capture list changed
	// (no-op while it is closed). Change-gated so editing the exclude
	// field is not disturbed by periodic rebuilds.
	if (wall->props_dirty.exchange(false, std::memory_order_relaxed))
		obs_source_update_properties(wall->context);
	// Bearings depend on the live screen geometry and the Display Wall
	// layout; refresh the filters when either moves.
	{
		const float half_w = g_device.screen_half_width_m.load(
			std::memory_order_relaxed);
		const float dist = g_device.screen_distance_m.load(
			std::memory_order_relaxed);
		const float curve = g_device.screen_curve.load(
			std::memory_order_relaxed);
		const uint32_t gen = nyan_real_wall_map_generation();
		if (std::fabs(half_w - wall->geo_half_w) > 0.005f ||
		    std::fabs(dist - wall->geo_dist) > 0.005f ||
		    std::fabs(curve - wall->geo_curve) > 0.005f ||
		    gen != wall->geo_map_gen) {
			wall->geo_half_w = half_w;
			wall->geo_dist = dist;
			wall->geo_curve = curve;
			wall->geo_map_gen = gen;
			wall->filters_dirty.store(true,
						  std::memory_order_relaxed);
		}
	}
	std::lock_guard<std::mutex> lk(wall->children_mutex);
	if (wall->filters_dirty.exchange(false, std::memory_order_relaxed)) {
		for (auto &child : wall->children)
			update_child_filter(wall, child.get());
		for (auto &entry : wall->ws_streams)
			update_spatial_filter(wall, entry.second.filter,
					      entry.second.norm_x);
	}
	// Drop WS streams whose extension went away without a close (tab
	// crash, browser exit); a live stream pushes PCM continuously.
	const uint64_t now = os_gettime_ns();
	bool removed = false;
	for (auto it = wall->ws_streams.begin();
	     it != wall->ws_streams.end();) {
		if (it->second.last_rx_ns &&
		    now - it->second.last_rx_ns > WS_STREAM_TIMEOUT_NS) {
			blog(LOG_INFO,
			     "[obs-nyan-real-3dof] audio wall: ws stream timed out '%s'",
			     it->second.label.c_str());
			release_ws_stream_locked(wall, it->second);
			it = wall->ws_streams.erase(it);
			removed = true;
		} else {
			++it;
		}
	}
	if (removed) {
		rebuild_ws_exes_locked(wall);
		wall->props_dirty.store(true, std::memory_order_relaxed);
	}
	wall_add_active_children(wall);
}

void wall_show(void *data)
{
	auto *wall = static_cast<audio_wall_source *>(data);
	std::lock_guard<std::mutex> lk(wall->children_mutex);
	wall_add_active_children(wall);
}

void wall_hide(void *data)
{
	auto *wall = static_cast<audio_wall_source *>(data);
	std::lock_guard<std::mutex> lk(wall->children_mutex);
	wall_remove_active_children(wall);
}

void wall_enum_active(void *data, obs_source_enum_proc_t enum_callback,
		      void *param)
{
	auto *wall = static_cast<audio_wall_source *>(data);
	std::lock_guard<std::mutex> lk(wall->children_mutex);
	for (auto &child : wall->children) {
		if (child->source)
			enum_callback(wall->context, child->source, param);
	}
	for (auto &entry : wall->ws_streams) {
		if (entry.second.source)
			enum_callback(wall->context, entry.second.source,
				      param);
	}
}

} // namespace

void register_nyan_real_audio_wall_source()
{
	static obs_source_info ws_info = {};
	ws_info.id = WS_AUDIO_SOURCE_ID;
	ws_info.type = OBS_SOURCE_TYPE_INPUT;
	ws_info.output_flags = OBS_SOURCE_AUDIO | OBS_SOURCE_CAP_DISABLED |
			       OBS_SOURCE_DO_NOT_DUPLICATE;
	ws_info.get_name = ws_audio_get_name;
	ws_info.create = ws_audio_create;
	ws_info.destroy = ws_audio_destroy;
	obs_register_source(&ws_info);

	static obs_source_info info = {};
	info.id = "nyan_real_3dof_audio_wall";
	info.type = OBS_SOURCE_TYPE_INPUT;
	// No AUDIO/VIDEO flags: the wall is a manager. Its private children
	// capture and monitor the audio themselves; the wall only keeps them
	// active (enum_active_sources) and feeds their spatial filters.
	info.output_flags = OBS_SOURCE_DO_NOT_DUPLICATE;
	info.get_name = wall_get_name;
	info.create = wall_create;
	info.destroy = wall_destroy;
	info.get_defaults = wall_defaults;
	info.get_properties = wall_properties;
	info.update = wall_update;
	info.video_tick = wall_tick;
	info.show = wall_show;
	info.hide = wall_hide;
	info.enum_active_sources = wall_enum_active;
	info.enum_all_sources = wall_enum_active;
	info.icon_type = OBS_ICON_TYPE_AUDIO_OUTPUT;
	obs_register_source(&info);
}
