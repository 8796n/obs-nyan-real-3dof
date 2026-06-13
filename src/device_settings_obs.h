// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// OBS-side glue for device_manager settings: load/save through obs_data and the
// recenter hotkey callback. The core device_manager carries no OBS dependency;
// this layer translates OBS settings to/from its fields.
#pragma once

#include <obs-module.h>

#include "device_manager.h"

void manager_apply_settings(device_manager *f, obs_data_t *settings);
void manager_save_load(obs_data_t *save_data, bool saving, void *private_data);
void recenter_hotkey(void *data, obs_hotkey_id id, obs_hotkey_t *hotkey,
		     bool pressed);
