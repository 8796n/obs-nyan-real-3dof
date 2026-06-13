// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Device registry: built-in table plus user devices.json entries, and the
// EDID identifiers of the glasses' display panels.
#pragma once

#include <vector>

#include "nyan_types.h"

// Built once in obs_module_load (user entries first, then built-ins) and
// immutable afterwards, so every thread reads it without locking.
extern std::vector<device_entry> g_device_registry;

const model_profile &profile_for(model_id m);
transport_traits traits_for(imu_transport t);
void init_device_registry();
