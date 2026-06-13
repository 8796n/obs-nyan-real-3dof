// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Keeps the mouse cursor off the EDID-identified glasses display while a
// projector covers it. A low-level mouse hook tunnels the cursor through to
// the display on the far side when one exists, and acts as a wall otherwise,
// so the desktop stays fully reachable regardless of the monitor layout.
#pragma once

// UI-thread only (the hook needs the caller's message loop). Call every dock
// poll with the current setting and glasses-display rect; installs/removes
// the hook on transitions. Disabling, losing the display, or failing to push
// an already-inside cursor out all lower the fence.
void cursor_fence_update(bool enabled, bool has_rect, long left, long top,
			 long right, long bottom);
void cursor_fence_shutdown();
