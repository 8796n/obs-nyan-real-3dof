// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// WASAPI render-endpoint master volume / mute by endpoint id
// (IAudioEndpointVolume). OBS-free Win32 shared by the dock so the OBS plugin
// and the standalone app control the chosen output device's volume the same way.
// COM must already be initialized on the calling thread (the Qt UI thread is).
#pragma once

#include <string>

// Master volume scalar 0..1 of the endpoint; <0 on failure.
float endpoint_volume_get(const std::string &endpoint_id);
// Set master volume scalar (clamped to 0..1). false on failure.
bool endpoint_volume_set(const std::string &endpoint_id, float scalar);
// Mute state; *ok = false when the query failed (return value then meaningless).
bool endpoint_volume_get_mute(const std::string &endpoint_id, bool *ok);
// Set mute. false on failure.
bool endpoint_volume_set_mute(const std::string &endpoint_id, bool mute);

// Id of the current Windows default render endpoint (eConsole role); "" on
// failure. Used to warn when the spatial-audio output is the system default
// (raw + spatialized would double up).
std::string default_render_endpoint_id();
