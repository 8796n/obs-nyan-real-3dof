// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "cursor_fence.h"

#include <windows.h>

#include <atomic>

#include "nyan_log.h"

// Marks our own injected warp events so the hook passes them through instead
// of fencing them again.
constexpr ULONG_PTR FENCE_WARP_SIG = 0x6E79616E; // "nyan"

static HHOOK g_fence_hook = nullptr;
static std::atomic<bool> g_fence_rect_valid{false};
static std::atomic<LONG> g_fence_left{0};
static std::atomic<LONG> g_fence_top{0};
static std::atomic<LONG> g_fence_right{0};
static std::atomic<LONG> g_fence_bottom{0};

static RECT fence_rect()
{
	return {g_fence_left.load(std::memory_order_relaxed),
		g_fence_top.load(std::memory_order_relaxed),
		g_fence_right.load(std::memory_order_relaxed),
		g_fence_bottom.load(std::memory_order_relaxed)};
}

static void send_cursor_warp(POINT pt)
{
	const int vx = GetSystemMetrics(SM_XVIRTUALSCREEN);
	const int vy = GetSystemMetrics(SM_YVIRTUALSCREEN);
	const int vw = GetSystemMetrics(SM_CXVIRTUALSCREEN);
	const int vh = GetSystemMetrics(SM_CYVIRTUALSCREEN);
	if (vw <= 1 || vh <= 1)
		return;
	INPUT in = {};
	in.type = INPUT_MOUSE;
	in.mi.dx = static_cast<LONG>((static_cast<LONGLONG>(pt.x - vx) * 65535) /
				     (vw - 1));
	in.mi.dy = static_cast<LONG>((static_cast<LONGLONG>(pt.y - vy) * 65535) /
				     (vh - 1));
	in.mi.dwFlags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE |
			MOUSEEVENTF_VIRTUALDESK;
	in.mi.dwExtraInfo = FENCE_WARP_SIG;
	SendInput(1, &in, sizeof(in));
}

enum class fence_side { left, right, top, bottom, inside };

static fence_side entry_side(POINT from, const RECT &r)
{
	if (from.x < r.left)
		return fence_side::left;
	if (from.x >= r.right)
		return fence_side::right;
	if (from.y < r.top)
		return fence_side::top;
	if (from.y >= r.bottom)
		return fence_side::bottom;
	return fence_side::inside;
}

// Project the blocked movement across the fence rect; true (tunnel) when a
// monitor exists on the far side, false (wall) when the glasses display sits
// at the edge of the layout in that direction.
static bool tunnel_exit_point(fence_side side, POINT to, const RECT &r,
			      POINT &out)
{
	POINT probe = to;
	switch (side) {
	case fence_side::left:
		probe.x = r.right;
		break;
	case fence_side::right:
		probe.x = r.left - 1;
		break;
	case fence_side::top:
		probe.y = r.bottom;
		break;
	case fence_side::bottom:
		probe.y = r.top - 1;
		break;
	case fence_side::inside:
		return false; // ejection handles this
	}
	if (!MonitorFromPoint(probe, MONITOR_DEFAULTTONULL))
		return false;
	out = probe;
	return true;
}

// Clamp the blocked movement to just outside the entry edge, keeping the
// tangential component so the cursor slides along the wall instead of
// sticking on diagonal movement.
static bool wall_slide_point(fence_side side, POINT to, const RECT &r,
			     POINT &out)
{
	POINT slide = to;
	switch (side) {
	case fence_side::left:
		slide.x = r.left - 1;
		break;
	case fence_side::right:
		slide.x = r.right;
		break;
	case fence_side::top:
		slide.y = r.top - 1;
		break;
	case fence_side::bottom:
		slide.y = r.bottom;
		break;
	case fence_side::inside:
		return false;
	}
	if (!MonitorFromPoint(slide, MONITOR_DEFAULTTONULL))
		return false;
	out = slide;
	return true;
}

static LRESULT CALLBACK fence_hook_proc(int code, WPARAM wparam, LPARAM lparam)
{
	if (code != HC_ACTION ||
	    !g_fence_rect_valid.load(std::memory_order_relaxed))
		return CallNextHookEx(nullptr, code, wparam, lparam);
	const auto *ms = reinterpret_cast<const MSLLHOOKSTRUCT *>(lparam);
	if (ms->dwExtraInfo == FENCE_WARP_SIG)
		return CallNextHookEx(nullptr, code, wparam, lparam);
	const RECT r = fence_rect();
	if (!PtInRect(&r, ms->pt))
		return CallNextHookEx(nullptr, code, wparam, lparam);

	// The event would put the cursor onto the glasses display.
	POINT cur = {};
	if (!GetCursorPos(&cur) || PtInRect(&r, cur)) {
		// Already inside (fence just rose / display hotplug). Let
		// events through so the dock-poll ejection can move it out.
		return CallNextHookEx(nullptr, code, wparam, lparam);
	}
	const fence_side side = entry_side(cur, r);
	POINT warp_pt = {};
	if (tunnel_exit_point(side, ms->pt, r, warp_pt) ||
	    wall_slide_point(side, ms->pt, r, warp_pt))
		send_cursor_warp(warp_pt);
	return 1; // swallow the original event: tunnel, slide or stop
}

// Push an already-inside cursor onto a normal display. False when no normal
// display is reachable, in which case the fence must stay down to avoid
// trapping the cursor.
static bool eject_cursor(const RECT &r)
{
	POINT cur = {};
	if (!GetCursorPos(&cur) || !PtInRect(&r, cur))
		return true;
	const POINT probes[] = {{r.left - 1, cur.y},
				{r.right, cur.y},
				{cur.x, r.top - 1},
				{cur.x, r.bottom}};
	for (const POINT &probe : probes) {
		if (MonitorFromPoint(probe, MONITOR_DEFAULTTONULL)) {
			send_cursor_warp(probe);
			return true;
		}
	}
	const POINT center = {GetSystemMetrics(SM_CXSCREEN) / 2,
			      GetSystemMetrics(SM_CYSCREEN) / 2};
	if (!PtInRect(&r, center) &&
	    MonitorFromPoint(center, MONITOR_DEFAULTTONULL)) {
		send_cursor_warp(center);
		return true;
	}
	return false;
}

static void lower_fence()
{
	g_fence_rect_valid.store(false, std::memory_order_relaxed);
	if (g_fence_hook) {
		UnhookWindowsHookEx(g_fence_hook);
		g_fence_hook = nullptr;
		nyan_log(NYAN_LOG_INFO, "[obs-nyan-real-3dof] cursor fence disabled");
	}
}

void cursor_fence_update(bool enabled, bool has_rect, long left, long top,
			 long right, long bottom)
{
	if (!enabled || !has_rect || right <= left || bottom <= top) {
		lower_fence();
		return;
	}

	g_fence_left.store(left, std::memory_order_relaxed);
	g_fence_top.store(top, std::memory_order_relaxed);
	g_fence_right.store(right, std::memory_order_relaxed);
	g_fence_bottom.store(bottom, std::memory_order_relaxed);

	const RECT r = {left, top, right, bottom};
	if (!eject_cursor(r)) {
		// No normal display to escape to; a fence would trap the
		// cursor on the glasses display.
		g_fence_rect_valid.store(false, std::memory_order_relaxed);
		return;
	}

	g_fence_rect_valid.store(true, std::memory_order_relaxed);
	if (!g_fence_hook) {
		g_fence_hook = SetWindowsHookExW(WH_MOUSE_LL, fence_hook_proc,
						 GetModuleHandleW(nullptr), 0);
		if (g_fence_hook)
			nyan_log(NYAN_LOG_INFO,
			     "[obs-nyan-real-3dof] cursor fence enabled "
			     "(%ld,%ld)-(%ld,%ld)",
			     left, top, right, bottom);
		else
			nyan_log(NYAN_LOG_WARNING,
			     "[obs-nyan-real-3dof] cursor fence hook failed "
			     "(error %lu)",
			     GetLastError());
	}
}

void cursor_fence_shutdown()
{
	lower_fence();
}
