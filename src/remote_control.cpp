// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
#include "remote_control.h"

#include <obs-module.h>
#include <util/platform.h>

#include <winsock2.h>
#include <iphlpapi.h>
#include <windows.h>
#include <wincrypt.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "device_manager.h"
#include "ws_server.h"

namespace {

// The page heartbeats every 4 s while visible; three misses mean the phone
// is gone (screen locked, Wi-Fi dropped) and the connection is evicted so
// the dock's "connected" display stays honest. TCP alone would keep a dead
// peer "connected" for minutes.
constexpr uint64_t REMOTE_IDLE_TIMEOUT_MS = 12000;

struct remote_state {
	ws_server server;
	// Token the running server was started with; a token change (reset to
	// defaults) restarts the server so old QR codes stop working.
	std::string active_token;
	// Throttle failed bind retries so a busy port does not warn every
	// dock poll.
	uint64_t last_start_attempt_ms = 0;

	std::mutex conns_mutex;
	// Established connections -> last message tick (GetTickCount64 ms).
	std::map<uint64_t, uint64_t> conns;

	// Distance last pushed to clients; the dock slider moves it too, so
	// the poll re-broadcasts whenever it drifts.
	std::atomic<float> sent_distance{-1.0f};

	// Sub-pixel/sub-notch accumulators. One set is enough: a second
	// simultaneous controller fighting over one cursor is not a real use
	// case, but the mutex keeps the math sane if it happens.
	std::mutex inject_mutex;
	double acc_x = 0.0;
	double acc_y = 0.0;
	double acc_wheel = 0.0;

	// Buttons currently held down by remote commands (left/right/middle).
	// A dropped connection (phone locks its screen mid-drag) would leave
	// the OS button stuck otherwise; on_close releases what is still down.
	std::atomic<bool> button_down[3] = {};

	// LAN address cache; adapter enumeration is too heavy for every poll.
	std::mutex url_mutex;
	std::string cached_ip;
	uint64_t cached_ip_ms = 0;
};

remote_state g_remote;

// Best local IPv4 for the QR code URL: prefer adapters with a default
// gateway (a real LAN, not a virtual switch), then private ranges. Returns
// "" when only loopback/link-local addresses exist.
std::string pick_lan_ipv4()
{
	// INCLUDE_GATEWAYS matters: FirstGatewayAddress stays null without it
	// and the scoring below would treat every adapter as a virtual switch.
	const ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
			    GAA_FLAG_SKIP_DNS_SERVER |
			    GAA_FLAG_INCLUDE_GATEWAYS;
	ULONG len = 16 * 1024;
	std::vector<uint8_t> buf(len);
	auto *addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buf.data());
	ULONG err = GetAdaptersAddresses(AF_INET, flags, nullptr, addrs, &len);
	if (err == ERROR_BUFFER_OVERFLOW) {
		buf.resize(len);
		addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buf.data());
		err = GetAdaptersAddresses(AF_INET, flags, nullptr, addrs,
					   &len);
	}
	if (err != NO_ERROR)
		return "";

	std::string best;
	int best_score = -1;
	for (auto *a = addrs; a; a = a->Next) {
		if (a->OperStatus != IfOperStatusUp ||
		    a->IfType == IF_TYPE_SOFTWARE_LOOPBACK)
			continue;
		for (auto *ua = a->FirstUnicastAddress; ua; ua = ua->Next) {
			if (ua->Address.lpSockaddr->sa_family != AF_INET)
				continue;
			const auto *sin = reinterpret_cast<sockaddr_in *>(
				ua->Address.lpSockaddr);
			const uint32_t ip =
				ntohl(sin->sin_addr.S_un.S_addr);
			const uint8_t b0 = ip >> 24, b1 = (ip >> 16) & 0xFF;
			if (b0 == 127 || (b0 == 169 && b1 == 254))
				continue;
			int score = 0;
			if (a->FirstGatewayAddress)
				score += 2;
			if (b0 == 192 && b1 == 168)
				score += 1;
			else if (b0 == 10)
				score += 1;
			else if (b0 == 172 && b1 >= 16 && b1 <= 31)
				score += 1;
			if (score > best_score) {
				char text[16];
				snprintf(text, sizeof(text), "%u.%u.%u.%u", b0,
					 b1, (ip >> 8) & 0xFF, ip & 0xFF);
				best = text;
				best_score = score;
			}
		}
	}
	return best;
}

// 60-bit random token in base32 (URL-safe, no escaping anywhere).
std::string generate_token()
{
	static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";
	uint8_t raw[12] = {};
	HCRYPTPROV prov = 0;
	if (CryptAcquireContextW(&prov, nullptr, nullptr, PROV_RSA_FULL,
				 CRYPT_VERIFYCONTEXT)) {
		CryptGenRandom(prov, sizeof(raw), raw);
		CryptReleaseContext(prov, 0);
	}
	std::string token;
	for (uint8_t b : raw)
		token += alphabet[b & 31];
	return token;
}

std::string ensure_token()
{
	std::lock_guard<std::mutex> lk(g_device.settings_mutex);
	if (g_device.remote_token.empty())
		g_device.remote_token = generate_token();
	return g_device.remote_token;
}

// "/ws?t=TOKEN" - the only valid upgrade target.
bool upgrade_target_ok(const std::string &target, const std::string &token)
{
	const size_t q = target.find('?');
	if (target.substr(0, q) != "/ws" || q == std::string::npos)
		return false;
	std::string query = target.substr(q + 1);
	size_t pos = 0;
	while (pos <= query.size()) {
		size_t amp = query.find('&', pos);
		if (amp == std::string::npos)
			amp = query.size();
		const std::string param = query.substr(pos, amp - pos);
		if (param.rfind("t=", 0) == 0)
			return !token.empty() &&
			       param.compare(2, std::string::npos, token) == 0;
		pos = amp + 1;
	}
	return false;
}

std::string load_page()
{
	char *path = obs_module_file("remote.html");
	if (!path)
		return "";
	char *text = os_quick_read_utf8_file(path);
	std::string body = text ? text : "";
	bfree(text);
	bfree(path);
	return body;
}

std::string state_json()
{
	const double dist =
		g_device.screen_distance_m.load(std::memory_order_relaxed);
	char json[64];
	snprintf(json, sizeof(json), "{\"t\":\"state\",\"dist\":%.3f}", dist);
	return json;
}

void broadcast_state()
{
	const std::string json = state_json();
	g_remote.sent_distance.store(
		g_device.screen_distance_m.load(std::memory_order_relaxed),
		std::memory_order_relaxed);
	std::vector<uint64_t> conns;
	{
		std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
		for (const auto &kv : g_remote.conns)
			conns.push_back(kv.first);
	}
	for (uint64_t conn : conns)
		g_remote.server.send_text(conn, json);
}

// Pointer moves ride MOUSEEVENTF_ABSOLUTE (current position + delta) instead
// of relative moves, so Windows' pointer acceleration never warps the
// touchpad mapping. No dwExtraInfo signature on purpose: the cursor fence
// must keep treating these like real mouse moves.
void inject_move(double dx, double dy)
{
	// Bound a garbage message; a real flick is well under this.
	dx = clampd(dx, -4096.0, 4096.0);
	dy = clampd(dy, -4096.0, 4096.0);
	long ix, iy;
	{
		std::lock_guard<std::mutex> lk(g_remote.inject_mutex);
		g_remote.acc_x += dx;
		g_remote.acc_y += dy;
		ix = std::lround(g_remote.acc_x);
		iy = std::lround(g_remote.acc_y);
		g_remote.acc_x -= ix;
		g_remote.acc_y -= iy;
	}
	if (!ix && !iy)
		return;
	POINT cur = {};
	if (!GetCursorPos(&cur))
		return;
	const int vx = GetSystemMetrics(SM_XVIRTUALSCREEN);
	const int vy = GetSystemMetrics(SM_YVIRTUALSCREEN);
	const int vw = GetSystemMetrics(SM_CXVIRTUALSCREEN);
	const int vh = GetSystemMetrics(SM_CYVIRTUALSCREEN);
	if (vw <= 1 || vh <= 1)
		return;
	const long tx = std::min<long>(std::max<long>(cur.x + ix, vx),
				       vx + vw - 1);
	const long ty = std::min<long>(std::max<long>(cur.y + iy, vy),
				       vy + vh - 1);
	INPUT in = {};
	in.type = INPUT_MOUSE;
	in.mi.dx = static_cast<LONG>((static_cast<LONGLONG>(tx - vx) * 65535) /
				     (vw - 1));
	in.mi.dy = static_cast<LONG>((static_cast<LONGLONG>(ty - vy) * 65535) /
				     (vh - 1));
	in.mi.dwFlags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE |
			MOUSEEVENTF_VIRTUALDESK;
	SendInput(1, &in, sizeof(in));
}

void inject_button(int button, bool down)
{
	INPUT in = {};
	in.type = INPUT_MOUSE;
	if (button == 1)
		in.mi.dwFlags = down ? MOUSEEVENTF_RIGHTDOWN
				     : MOUSEEVENTF_RIGHTUP;
	else if (button == 2)
		in.mi.dwFlags = down ? MOUSEEVENTF_MIDDLEDOWN
				     : MOUSEEVENTF_MIDDLEUP;
	else
		in.mi.dwFlags = down ? MOUSEEVENTF_LEFTDOWN
				     : MOUSEEVENTF_LEFTUP;
	g_remote.button_down[button == 1 ? 1 : (button == 2 ? 2 : 0)].store(
		down, std::memory_order_relaxed);
	SendInput(1, &in, sizeof(in));
}

// Lift whatever the remote still holds (connection loss, server stop).
void release_held_buttons()
{
	for (int b = 0; b < 3; b++) {
		if (g_remote.button_down[b].load(std::memory_order_relaxed))
			inject_button(b, false);
	}
}

// Fractional notches accumulate into high-resolution wheel events, so a slow
// two-finger scroll still moves apps that only react to whole detents late
// rather than never.
void inject_wheel(double notches)
{
	notches = clampd(notches, -40.0, 40.0);
	long amount;
	{
		std::lock_guard<std::mutex> lk(g_remote.inject_mutex);
		g_remote.acc_wheel += notches * WHEEL_DELTA;
		amount = std::lround(g_remote.acc_wheel);
		g_remote.acc_wheel -= amount;
	}
	if (!amount)
		return;
	INPUT in = {};
	in.type = INPUT_MOUSE;
	in.mi.dwFlags = MOUSEEVENTF_WHEEL;
	in.mi.mouseData = static_cast<DWORD>(amount);
	SendInput(1, &in, sizeof(in));
}

void handle_message(uint64_t conn, obs_data_t *msg)
{
	// Every message is a liveness proof, including the page's bare
	// {"t":"hb"} heartbeats.
	{
		std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
		auto it = g_remote.conns.find(conn);
		if (it != g_remote.conns.end())
			it->second = GetTickCount64();
	}
	const char *type = obs_data_get_string(msg, "t");
	if (!type)
		return;
	if (strcmp(type, "mv") == 0) {
		inject_move(obs_data_get_double(msg, "dx"),
			    obs_data_get_double(msg, "dy"));
	} else if (strcmp(type, "btn") == 0) {
		inject_button(static_cast<int>(obs_data_get_int(msg, "b")),
			      obs_data_get_bool(msg, "d"));
	} else if (strcmp(type, "whl") == 0) {
		inject_wheel(obs_data_get_double(msg, "n"));
	} else if (strcmp(type, "dist") == 0) {
		manager_step_screen_distance(
			&g_device,
			clampd(obs_data_get_double(msg, "s"), -100.0, 100.0));
		broadcast_state();
	} else if (strcmp(type, "recenter") == 0) {
		manager_recenter(&g_device);
	} else if (strcmp(type, "recal") == 0) {
		manager_recalibrate(&g_device);
	}
}

void start_server(uint16_t port, const std::string &token)
{
	ws_server_callbacks cb;
	cb.on_http_get = [](const std::string &target) -> std::string {
		const std::string path = target.substr(0, target.find('?'));
		if (path != "/" && path != "/index.html")
			return "";
		return load_page();
	};
	cb.on_ws_accept = [token](const std::string &target) {
		return upgrade_target_ok(target, token);
	};
	cb.on_open = [](uint64_t conn) {
		{
			std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
			g_remote.conns[conn] = GetTickCount64();
		}
		g_remote.server.send_text(conn, state_json());
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] remote: client connected (%d active)",
		     remote_control_client_count());
	};
	cb.on_close = [](uint64_t conn) {
		{
			std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
			g_remote.conns.erase(conn);
		}
		release_held_buttons();
	};
	cb.on_text = handle_message;

	{
		std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
		g_remote.conns.clear();
	}
	if (g_remote.server.start(port, std::move(cb), /*lan=*/true))
		g_remote.active_token = token;
}

} // namespace

void remote_control_sync()
{
	const bool want =
		g_device.remote_enabled.load(std::memory_order_relaxed);
	if (!want) {
		if (g_remote.server.running()) {
			g_remote.server.stop();
			release_held_buttons();
			std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
			g_remote.conns.clear();
		}
		return;
	}

	int port = g_device.remote_port.load(std::memory_order_relaxed);
	if (port < 1024 || port > 65535)
		port = DEFAULT_REMOTE_PORT;
	const std::string token = ensure_token();

	if (!g_remote.server.running() ||
	    g_remote.server.port() != static_cast<uint16_t>(port) ||
	    g_remote.active_token != token) {
		const uint64_t now = GetTickCount64();
		if (now - g_remote.last_start_attempt_ms < 3000)
			return;
		g_remote.last_start_attempt_ms = now;
		start_server(static_cast<uint16_t>(port), token);
		return;
	}

	// Evict connections whose heartbeats stopped; their threads run the
	// normal on_close teardown (button release, conns erase).
	{
		const uint64_t now = GetTickCount64();
		std::vector<uint64_t> stale;
		{
			std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
			for (const auto &kv : g_remote.conns) {
				if (now - kv.second > REMOTE_IDLE_TIMEOUT_MS)
					stale.push_back(kv.first);
			}
		}
		for (uint64_t conn : stale)
			g_remote.server.close_conn(conn);
	}

	// The dock slider (or a settings load) moved the distance: keep the
	// phones' readout in sync.
	const float dist =
		g_device.screen_distance_m.load(std::memory_order_relaxed);
	if (std::fabs(dist - g_remote.sent_distance.load(
				     std::memory_order_relaxed)) > 1e-4f)
		broadcast_state();
}

void remote_control_shutdown()
{
	g_remote.server.stop();
	release_held_buttons();
	std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
	g_remote.conns.clear();
}

std::string remote_control_url()
{
	if (!g_remote.server.running())
		return "";
	std::string ip;
	{
		std::lock_guard<std::mutex> lk(g_remote.url_mutex);
		const uint64_t now = GetTickCount64();
		if (now - g_remote.cached_ip_ms > 3000 ||
		    g_remote.cached_ip_ms == 0) {
			g_remote.cached_ip = pick_lan_ipv4();
			g_remote.cached_ip_ms = now;
		}
		ip = g_remote.cached_ip;
	}
	if (ip.empty())
		return "";
	std::string token;
	{
		std::lock_guard<std::mutex> lk(g_device.settings_mutex);
		token = g_device.remote_token;
	}
	if (token.empty())
		return "";
	return "http://" + ip + ":" +
	       std::to_string(g_remote.server.port()) + "/?t=" + token;
}

int remote_control_client_count()
{
	std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
	return static_cast<int>(g_remote.conns.size());
}
