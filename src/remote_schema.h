// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Single definition table of the dock controls mirrored on the phone
// remote's settings screen. The page renders generically from the snapshot
// this builds, so adding/removing a mirrored control is a one-line change
// HERE and nowhere else (see CONTRIBUTING.md "スマホリモコンの設定ミラー").
#pragma once

typedef struct obs_data obs_data_t;

// Fills "sections" (array of {label, rows}) into cfg: the localized labels,
// types, current values, ranges, combo options and enabled states of every
// visible mirrored control. UI thread only - reads Win32 display info and
// enumerates audio endpoints exactly like the dock's poll does.
void remote_schema_build_cfg(obs_data_t *cfg);

// Applies a {"t":"set","k":...,"v":...} message from the page through the
// same table (whitelist + range clamp). Any thread. False for unknown keys.
bool remote_schema_apply(obs_data_t *msg);
