// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "remote_control.h"

#include <winsock2.h>
#include <iphlpapi.h>
#include <windows.h>
#include <wincrypt.h>
// uiautomation.h needs the OLE/COM prerequisites (the `interface` macro and the
// base provider interfaces). WIN32_LEAN_AND_MEAN keeps <windows.h> from pulling
// them in, so include them explicitly here or UIAutomationCore.h fails to parse.
#include <ole2.h>
#include <oleacc.h>
#include <uiautomation.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "device_manager.h"
#include "math_util.h"
#include "nyan_host.h"
#include "nyan_json.h"
#include "nyan_log.h"
#include "nyan_paths.h"
#include "remote_schema.h"
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

	// Last settings-mirror snapshot pushed to clients. Built by the sync
	// poll (UI thread - the schema reads display info and enumerates
	// audio endpoints like the dock does) and re-sent whenever the
	// serialized form changes, so dock-side edits show up on the phone.
	std::mutex cfg_mutex;
	std::string cfg_cache;

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

	// Focus watch (UI Automation): true while the host's focused control is a
	// typable text field, pushed to the phone in state_json so the remote can
	// light its keyboard button. The watcher only runs while a phone is
	// connected, so an idle remote puts no accessibility load on other apps.
	std::atomic<bool> edit_focused{false};
	std::thread focus_thread;
	std::atomic<bool> focus_running{false};
	HANDLE focus_wake = nullptr; // manual signal: tells the thread to stop
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

// 60-bit random token in base32 (URL-safe, no escaping anywhere). Returns ""
// when the OS CSPRNG is unavailable - the caller must then refuse to start
// the server rather than fall back to a guessable token (fail secure).
std::string generate_token()
{
	static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";
	uint8_t raw[12] = {};
	HCRYPTPROV prov = 0;
	bool ok = false;
	if (CryptAcquireContextW(&prov, nullptr, nullptr, PROV_RSA_FULL,
				 CRYPT_VERIFYCONTEXT)) {
		ok = CryptGenRandom(prov, sizeof(raw), raw);
		CryptReleaseContext(prov, 0);
	}
	if (!ok) {
		nyan_log(NYAN_LOG_WARNING,
			 "[obs-nyan-real-3dof] remote: CSPRNG unavailable, refusing to start (no secure token)");
		return "";
	}
	std::string token;
	for (uint8_t b : raw)
		token += alphabet[b & 31];
	return token;
}

// Returns the persisted token, generating one on first use. May return ""
// when the CSPRNG is unavailable; the caller treats that as "do not start".
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

// True when host (the Host header value, possibly "1.2.3.4:8797") is a bare
// IPv4 literal. The QR URL always uses the LAN IP, so legitimate phones pass;
// a DNS-rebinding attacker reaches us under their domain name, which fails -
// closing the hole where a malicious site could fetch the page cross-origin.
bool host_is_ipv4_literal(const std::string &host)
{
	const std::string h = host.substr(0, host.find(':')); // drop :port
	if (h.empty())
		return false;
	int octets = 1;
	int digits = 0;
	for (char c : h) {
		if (c == '.') {
			if (digits == 0)
				return false; // empty octet ("1..2", ".1")
			octets++;
			digits = 0;
		} else if (c >= '0' && c <= '9') {
			digits++;
		} else {
			return false; // a letter: a hostname, not an IP literal
		}
	}
	return octets == 4 && digits > 0;
}

std::string load_page()
{
	std::string path = nyan_asset_path("remote.html");
	if (path.empty())
		return "";
	std::ifstream f(path, std::ios::binary);
	if (!f)
		return "";
	std::ostringstream ss;
	ss << f.rdbuf();
	std::string body = ss.str();
	// Drop a leading UTF-8 BOM so the served page starts at '<' (the old
	// os_quick_read_utf8_file did this); browsers tolerate it either way.
	if (body.size() >= 3 && static_cast<unsigned char>(body[0]) == 0xEF &&
	    static_cast<unsigned char>(body[1]) == 0xBB &&
	    static_cast<unsigned char>(body[2]) == 0xBF)
		body.erase(0, 3);
	return body;
}

// Depth the phone displays: the base screen distance plus how far the gaze
// dolly has walked forward (z is negative toward the screen).
float effective_distance()
{
	const float d =
		g_device.screen_distance_m.load(std::memory_order_relaxed);
	const float oz =
		g_device.viewer_offset_z.load(std::memory_order_relaxed);
	return static_cast<float>(clampd(d + oz, 0.05, MAX_DOLLY_GAP_M));
}

std::string state_json()
{
	// lang: the page chrome follows OBS's language once connected, so the
	// touchpad UI and the settings mirror (server-localized labels) agree.
	const char *locale = nyan_locale();
	const bool ja = locale && strncmp(locale, "ja", 2) == 0;
	// edit: the host's focused control is a typable field (UIA focus watch),
	// so the phone lights its keyboard button without auto-opening anything.
	const bool edit = g_remote.edit_focused.load(std::memory_order_relaxed);
	char json[96];
	snprintf(json, sizeof(json),
		 "{\"t\":\"state\",\"dist\":%.3f,\"lang\":\"%s\",\"edit\":%s}",
		 effective_distance(), ja ? "ja" : "en", edit ? "true" : "false");
	return json;
}

void broadcast_text(const std::string &json)
{
	std::vector<uint64_t> conns;
	{
		std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
		for (const auto &kv : g_remote.conns)
			conns.push_back(kv.first);
	}
	for (uint64_t conn : conns)
		g_remote.server.send_text(conn, json);
}

void broadcast_state()
{
	g_remote.sent_distance.store(effective_distance(),
				     std::memory_order_relaxed);
	broadcast_text(state_json());
}

std::string build_cfg_json()
{
	nyan_json cfg = nyan_json::object();
	cfg["t"] = "cfg";
	remote_schema_build_cfg(cfg);
	return cfg.dump();
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

// Puts Unicode text on the clipboard (CF_UNICODETEXT). Returns false if the
// clipboard could not be opened or the allocation failed.
bool set_clipboard_text(const std::wstring &w)
{
	if (!OpenClipboard(nullptr))
		return false;
	bool ok = false;
	const size_t bytes = (w.size() + 1) * sizeof(wchar_t);
	if (HGLOBAL h = GlobalAlloc(GMEM_MOVEABLE, bytes)) {
		if (void *p = GlobalLock(h)) {
			memcpy(p, w.c_str(), bytes);
			GlobalUnlock(h);
			EmptyClipboard();
			if (SetClipboardData(CF_UNICODETEXT, h))
				ok = true; // clipboard now owns h
			else
				GlobalFree(h);
		} else {
			GlobalFree(h);
		}
	}
	CloseClipboard();
	return ok;
}

// Types a committed string at the host's focused control via the clipboard +
// Ctrl-V, NOT synthesized keystrokes. KEYEVENTF_UNICODE characters get captured
// by an active host IME - a Japanese IME in kana mode tries to *convert* the
// injected hiragana, garbling the text. A paste inserts the literal characters
// whatever the IME state, and works across native, browser and Electron
// (Chromium) fields alike. The phone's own IME is the only one composing. The
// previous clipboard text is restored shortly after the paste reads it.
void inject_text(const std::string &utf8)
{
	if (utf8.empty())
		return;
	// Bound a garbage/oversized message; a real edit is well under this.
	const std::string s = utf8.size() > 4096 ? utf8.substr(0, 4096) : utf8;
	int wlen = MultiByteToWideChar(CP_UTF8, 0, s.c_str(),
				       static_cast<int>(s.size()), nullptr, 0);
	if (wlen <= 0)
		return;
	std::wstring w(static_cast<size_t>(wlen), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()),
			    &w[0], wlen);

	// Save the current clipboard text (best effort - non-text contents like
	// an image cannot be preserved and are lost).
	std::wstring prev;
	bool had_prev = false;
	if (OpenClipboard(nullptr)) {
		if (HANDLE h = GetClipboardData(CF_UNICODETEXT)) {
			if (const wchar_t *p =
				    static_cast<const wchar_t *>(GlobalLock(h))) {
				prev = p;
				had_prev = true;
				GlobalUnlock(h);
			}
		}
		CloseClipboard();
	}

	if (!set_clipboard_text(w))
		return;

	INPUT in[4] = {};
	in[0].type = INPUT_KEYBOARD;
	in[0].ki.wVk = VK_CONTROL;
	in[1].type = INPUT_KEYBOARD;
	in[1].ki.wVk = 'V';
	in[2] = in[1];
	in[2].ki.dwFlags = KEYEVENTF_KEYUP;
	in[3] = in[0];
	in[3].ki.dwFlags = KEYEVENTF_KEYUP;
	SendInput(4, in, sizeof(INPUT));

	// Restore the old clipboard once the target has had time to read the
	// paste (WM_PASTE is processed on the target's queue, async to us).
	if (had_prev) {
		std::thread([prev]() {
			Sleep(400);
			set_clipboard_text(prev);
		}).detach();
	}
}

// Editing/navigation keys the phone can send by name (the touchpad already
// covers clicks). Whitelisted so an unknown key string is a no-op rather than
// an arbitrary VK. The extended flag matters for the navigation cluster: some
// apps distinguish the real arrows/Home/End/Delete from their numpad twins.
struct vkey_entry {
	const char *name;
	WORD vk;
	bool ext;
};
constexpr vkey_entry VKEYS[] = {
	{"enter", VK_RETURN, false}, {"backspace", VK_BACK, false},
	{"tab", VK_TAB, false},      {"esc", VK_ESCAPE, false},
	{"del", VK_DELETE, true},    {"up", VK_UP, true},
	{"down", VK_DOWN, true},     {"left", VK_LEFT, true},
	{"right", VK_RIGHT, true},   {"home", VK_HOME, true},
	{"end", VK_END, true},
};

void inject_vkey(const std::string &name)
{
	const vkey_entry *e = nullptr;
	for (const auto &v : VKEYS) {
		if (name == v.name) {
			e = &v;
			break;
		}
	}
	if (!e)
		return;
	INPUT in[2] = {};
	in[0].type = INPUT_KEYBOARD;
	in[0].ki.wVk = e->vk;
	in[0].ki.dwFlags = e->ext ? KEYEVENTF_EXTENDEDKEY : 0;
	in[1] = in[0];
	in[1].ki.dwFlags |= KEYEVENTF_KEYUP;
	SendInput(2, in, sizeof(INPUT));
}

void handle_message(uint64_t conn, const nyan_json &msg)
{
	// Every message is a liveness proof, including the page's bare
	// {"t":"hb"} heartbeats.
	{
		std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
		auto it = g_remote.conns.find(conn);
		if (it != g_remote.conns.end())
			it->second = GetTickCount64();
	}
	const std::string type = nyan_json_get_string(msg, "t");
	if (type.empty())
		return;
	if (type == "mv") {
		inject_move(nyan_json_get_double(msg, "dx"),
			    nyan_json_get_double(msg, "dy"));
	} else if (type == "btn") {
		inject_button(static_cast<int>(nyan_json_get_int(msg, "b")),
			      nyan_json_get_bool(msg, "d"));
	} else if (type == "whl") {
		inject_wheel(nyan_json_get_double(msg, "n"));
	} else if (type == "text") {
		// Phone keyboard: a confirmed string typed at the host's focused
		// control (the user clicks the target field via the touchpad first).
		inject_text(nyan_json_get_string(msg, "s"));
	} else if (type == "key") {
		// Editing/navigation key by name (enter, backspace, arrows, ...).
		inject_vkey(nyan_json_get_string(msg, "k"));
	} else if (type == "dist") {
		// Gaze dolly: walk toward/away from the looked-at point, so
		// the zoom centers on it instead of the screen center.
		manager_dolly_step(
			&g_device,
			clampd(nyan_json_get_double(msg, "s"), -100.0, 100.0));
		broadcast_state();
	} else if (type == "focus") {
		// Wall focus picker: 0 = whole wall, else the chosen display's
		// Windows number. manager_set_focus_display clamps and bumps the
		// wall rebuild generation.
		manager_set_focus_display(
			&g_device, static_cast<int>(nyan_json_get_int(msg, "n")));
	} else if (type == "recenter") {
		manager_recenter(&g_device);
	} else if (type == "pose") {
		// 3DoF toggle on the main page: head-pose follow on/off (does not
		// touch the device connection). Recalibrate moved into the
		// settings mirror, routed through the schema's "set" path.
		g_device.pose_follow.store(nyan_json_get_bool(msg, "v"),
					   std::memory_order_relaxed);
	} else if (type == "set") {
		// Settings mirror: routed through the remote_schema table
		// (whitelist + clamp). The confirmed state comes back via the
		// poll's cfg push; the page renders optimistically meanwhile.
		remote_schema_apply(msg);
	}
}

// Whether a TextPattern element's content is read-only (a text view rather than
// an editor). Reads the IsReadOnly text attribute over the whole document range;
// a "mixed" value (vt != VT_BOOL) or a failed query is treated as editable, so
// we err toward offering input rather than withholding it on an odd app.
bool text_is_read_only(IUIAutomationElement *e)
{
	bool ro = false;
	IUIAutomationTextPattern *tp = nullptr;
	if (SUCCEEDED(e->GetCurrentPatternAs(
		    UIA_TextPatternId, __uuidof(IUIAutomationTextPattern),
		    reinterpret_cast<void **>(&tp))) &&
	    tp) {
		IUIAutomationTextRange *range = nullptr;
		if (SUCCEEDED(tp->get_DocumentRange(&range)) && range) {
			VARIANT v;
			VariantInit(&v);
			if (SUCCEEDED(range->GetAttributeValue(
				    UIA_IsReadOnlyAttributeId, &v)) &&
			    v.vt == VT_BOOL)
				ro = (v.boolVal == VARIANT_TRUE);
			VariantClear(&v);
			range->Release();
		}
		tp->Release();
	}
	return ro;
}

// True when a focused UIA element is something the user can type into:
//   1) a non-read-only ValuePattern (input / textarea / native Edit), or
//   2) a keyboard-focusable element exposing a TextPattern whose content is not
//      read-only - this catches contenteditable rich editors that come through
//      as Group/Custom with no ValuePattern (e.g. Electron apps like Claude
//      Desktop, which report ControlType Group + TextPattern).
// A read-only text view (TextPattern but IsReadOnly) is excluded so the hint
// never fires where typing has nowhere to land. Focus-driven, so static labels
// (which do not take keyboard focus) never reach here.
bool element_is_editable(IUIAutomationElement *e)
{
	if (!e)
		return false;

	IUIAutomationValuePattern *vp = nullptr;
	if (SUCCEEDED(e->GetCurrentPatternAs(
		    UIA_ValuePatternId, __uuidof(IUIAutomationValuePattern),
		    reinterpret_cast<void **>(&vp))) &&
	    vp) {
		BOOL ro = FALSE;
		vp->get_CurrentIsReadOnly(&ro);
		vp->Release();
		return !ro;
	}

	BOOL kbd = FALSE;
	e->get_CurrentIsKeyboardFocusable(&kbd);
	IUnknown *tp = nullptr;
	const bool has_text =
		SUCCEEDED(e->GetCurrentPattern(UIA_TextPatternId, &tp)) && tp;
	if (tp)
		tp->Release();
	if (kbd && has_text)
		return !text_is_read_only(e);

	CONTROLTYPEID ct = 0;
	e->get_CurrentControlType(&ct);
	return ct == UIA_EditControlTypeId || ct == UIA_DocumentControlTypeId;
}

// Recompute the edit-focused flag from the newly focused element and push a
// fresh state to the phones only when it flips, so a focus change inside the
// same field (or between two text fields) costs no traffic. Runs on a UIA
// worker thread; broadcast_state's reads are all atomic/mutex-guarded.
void update_edit_state(IUIAutomationElement *sender)
{
	const bool editable = element_is_editable(sender);
	if (g_remote.edit_focused.exchange(editable,
					   std::memory_order_relaxed) != editable)
		broadcast_state();
}

// COM sink for UIA focus-changed events. Single-purpose: every focus change in
// any app calls HandleFocusChangedEvent with the now-focused element.
class FocusHandler : public IUIAutomationFocusChangedEventHandler {
	LONG ref_ = 1;

public:
	ULONG STDMETHODCALLTYPE AddRef() override
	{
		return InterlockedIncrement(&ref_);
	}
	ULONG STDMETHODCALLTYPE Release() override
	{
		const LONG r = InterlockedDecrement(&ref_);
		if (r == 0)
			delete this;
		return r;
	}
	HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid,
						 void **ppv) override
	{
		if (riid == __uuidof(IUnknown) ||
		    riid == __uuidof(IUIAutomationFocusChangedEventHandler)) {
			*ppv = static_cast<IUIAutomationFocusChangedEventHandler *>(
				this);
			AddRef();
			return S_OK;
		}
		*ppv = nullptr;
		return E_NOINTERFACE;
	}
	HRESULT STDMETHODCALLTYPE
	HandleFocusChangedEvent(IUIAutomationElement *sender) override
	{
		update_edit_state(sender);
		return S_OK;
	}
};

// Lives on its own MTA thread: UIA delivers events on its internal pool, but
// the COM object and apartment must outlive the registration, so the thread
// holds them and parks on focus_wake until asked to stop.
void focus_watch_thread_proc()
{
	const HRESULT co = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
	IUIAutomation *uia = nullptr;
	FocusHandler *handler = nullptr;
	if (SUCCEEDED(CoCreateInstance(__uuidof(CUIAutomation), nullptr,
				       CLSCTX_INPROC_SERVER,
				       __uuidof(IUIAutomation),
				       reinterpret_cast<void **>(&uia))) &&
	    uia) {
		handler = new FocusHandler();
		if (FAILED(uia->AddFocusChangedEventHandler(nullptr, handler))) {
			handler->Release();
			handler = nullptr;
		}
	}
	WaitForSingleObject(g_remote.focus_wake, INFINITE);
	if (uia) {
		if (handler) {
			uia->RemoveFocusChangedEventHandler(handler);
			handler->Release();
		}
		uia->Release();
	}
	// A field may have been focused when we stop; clear the hint so it does
	// not stick on the phone after the watcher is gone.
	if (g_remote.edit_focused.exchange(false, std::memory_order_relaxed))
		broadcast_state();
	if (SUCCEEDED(co))
		CoUninitialize();
}

void focus_watch_start()
{
	if (g_remote.focus_running.load(std::memory_order_relaxed))
		return;
	g_remote.focus_wake = CreateEventW(nullptr, TRUE, FALSE, nullptr);
	if (!g_remote.focus_wake)
		return;
	g_remote.focus_running.store(true, std::memory_order_relaxed);
	g_remote.focus_thread = std::thread(focus_watch_thread_proc);
}

void focus_watch_stop()
{
	if (!g_remote.focus_running.load(std::memory_order_relaxed))
		return;
	SetEvent(g_remote.focus_wake);
	if (g_remote.focus_thread.joinable())
		g_remote.focus_thread.join();
	CloseHandle(g_remote.focus_wake);
	g_remote.focus_wake = nullptr;
	g_remote.focus_running.store(false, std::memory_order_relaxed);
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
	cb.on_check_host = host_is_ipv4_literal;
	cb.on_open = [](uint64_t conn) {
		// Single-user device: the newest authenticated session wins.
		// Evicting older sessions here means a half-dead connection
		// (phone slept mid-session) can never block a fresh scan.
		std::vector<uint64_t> evict;
		{
			std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
			for (const auto &kv : g_remote.conns)
				evict.push_back(kv.first);
			g_remote.conns[conn] = GetTickCount64();
		}
		for (uint64_t old : evict)
			g_remote.server.close_conn(old);
		g_remote.server.send_text(conn, state_json());
		// Possibly one poll stale; the sync push corrects it.
		std::string cfg;
		{
			std::lock_guard<std::mutex> lk(g_remote.cfg_mutex);
			cfg = g_remote.cfg_cache;
		}
		if (!cfg.empty())
			g_remote.server.send_text(conn, cfg);
		nyan_log(NYAN_LOG_INFO,
			 "[obs-nyan-real-3dof] remote: client connected%s",
			 evict.empty() ? "" : " (replacing the previous session)");
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
	// The remote follows both its own toggle and the master enable: when the
	// app is disabled (master off = fully idle) the LAN port closes too, for
	// the same "glasses/host freed" reason the other subsystems stop. OBS forces
	// connect_enabled true, so its remote is governed by remote_enabled alone.
	const bool want =
		g_device.remote_enabled.load(std::memory_order_relaxed) &&
		g_device.connect_enabled.load(std::memory_order_relaxed);
	if (!want) {
		focus_watch_stop();
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
	// Fail secure: without a real token the upgrade gate would accept any
	// client, so refuse to run rather than expose the command channel.
	if (token.empty()) {
		focus_watch_stop();
		if (g_remote.server.running()) {
			g_remote.server.stop();
			release_held_buttons();
			std::lock_guard<std::mutex> lk(g_remote.conns_mutex);
			g_remote.conns.clear();
		}
		return;
	}

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

	// The dock slider, a recenter (dolly reset) or a settings load moved
	// the effective depth: keep the phones' readout in sync.
	if (std::fabs(effective_distance() -
		      g_remote.sent_distance.load(std::memory_order_relaxed)) >
	    1e-4f)
		broadcast_state();

	// Watch the host's keyboard focus only while a phone is connected, so an
	// idle remote leaves other apps' accessibility untouched.
	if (remote_control_client_count() > 0)
		focus_watch_start();
	else
		focus_watch_stop();

	// Settings mirror: rebuild the snapshot while someone is connected
	// and push it when anything (dock edits, device state) changed.
	if (remote_control_client_count() > 0) {
		const std::string cfg = build_cfg_json();
		bool changed;
		{
			std::lock_guard<std::mutex> lk(g_remote.cfg_mutex);
			changed = cfg != g_remote.cfg_cache;
			if (changed)
				g_remote.cfg_cache = cfg;
		}
		if (changed)
			broadcast_text(cfg);
	}
}

void remote_control_rotate_token()
{
	// The user's intent is "revoke what is out there", so the old token
	// dies unconditionally: clear it and let the sync below either
	// generate a fresh one (normal case) or - with the CSPRNG broken -
	// take the fail-secure path and stop the server. Either way the
	// revocation holds; rotation never silently keeps the old token.
	{
		std::lock_guard<std::mutex> lk(g_device.settings_mutex);
		g_device.remote_token.clear();
	}
	// Skip the bind-retry throttle: a rotation should kill the running
	// sessions and show the fresh QR immediately.
	g_remote.last_start_attempt_ms = 0;
	remote_control_sync();
}

void remote_control_shutdown()
{
	focus_watch_stop();
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
