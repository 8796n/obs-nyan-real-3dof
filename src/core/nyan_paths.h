// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Core filesystem-path hook. Core code never knows where the host keeps its
// files; the host (OBS plugin or standalone app) installs resolvers that map a
// bare filename to an absolute path.
#pragma once

#include <string>

// Absolute path to a per-user config file (may not exist yet), e.g.
// "devices.json". Empty string if no resolver is installed.
std::string nyan_config_path(const char *filename);

// Absolute path to a bundled read-only asset, e.g. "remote.html". Empty
// string if not found or no resolver is installed.
std::string nyan_asset_path(const char *filename);

// Install the resolvers. Call once at startup before worker threads or any
// code that reads config/assets runs. Not thread-safe with concurrent lookups.
void nyan_set_path_resolvers(std::string (*config)(const char *filename),
			     std::string (*asset)(const char *filename));
