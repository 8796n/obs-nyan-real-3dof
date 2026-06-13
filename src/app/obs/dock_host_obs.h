// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// OBS-side host services for the shared dock: opens the glasses fullscreen via
// the OBS frontend source projector. Installs the provider into core's
// nyan_host at module load. No-op when built without the Qt dock.
#pragma once

void register_dock_host_obs();

// Creates the shared dock widget and registers it as an OBS frontend dock
// (no-op warning when built without Qt). Called from obs_module_load/unload.
void init_dock();
void shutdown_dock();
