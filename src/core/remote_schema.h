// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Single definition table of the dock controls mirrored on the phone
// remote's settings screen. The page renders generically from the snapshot
// this builds, so adding/removing a mirrored control is a one-line change
// HERE and nowhere else (see CONTRIBUTING.md "スマホリモコンの設定ミラー").
#pragma once

#include "nyan_json.h"

// Fills "sections" (array of {label, rows}) into cfg: the localized labels,
// types, current values, ranges, combo options and enabled states of every
// visible mirrored control. UI thread only - reads Win32 display info and
// enumerates audio endpoints exactly like the dock's poll does.
void remote_schema_build_cfg(nyan_json &cfg);

// Applies a {"t":"set","k":...,"v":...} message from the page through the
// same table (whitelist + range clamp). Any thread. False for unknown keys.
bool remote_schema_apply(const nyan_json &msg);

// Host-specific extra rows on the remote settings screen. The host appends its
// own sections (same JSON shape build_cfg produces) and applies "set" messages
// for its own keys, so host-only settings (the standalone's Display / Audio
// Wall) reach the phone without core knowing them. Unset by default - OBS keeps
// these per source, so it adds nothing and the mirror shows only shared (
// g_device) controls. build runs on the UI thread; apply on the WS thread.
void remote_schema_set_host_provider(void (*build)(nyan_json &sections),
				     bool (*apply)(const nyan_json &msg));
