// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Host-provided service hooks for core code that needs localization and audio
// endpoint enumeration without depending on OBS. The host (OBS plugin or
// standalone app) installs providers at startup; core code calls the wrappers.
#pragma once

#include <string>

// Localized text for a key (the dock's locale keys). Returns the key itself if
// no provider is installed, so labels degrade visibly rather than vanish.
const char *nyan_text(const char *key);
void nyan_set_text_provider(const char *(*provider)(const char *key));

// Current UI locale, e.g. "en-US" / "ja-JP". "en-US" if no provider.
const char *nyan_locale();
void nyan_set_locale_provider(const char *(*provider)());

// Enumerate audio output endpoints. The callback receives a friendly name and
// a stable endpoint id; returning false stops enumeration. No-op if no
// enumerator is installed. Signature mirrors libobs's monitoring-device enum
// so the OBS host can forward to obs_enum_audio_monitoring_devices.
using nyan_audio_output_cb = bool (*)(void *data, const char *name,
				      const char *id);
void nyan_enum_audio_outputs(nyan_audio_output_cb cb, void *data);
void nyan_set_audio_output_enumerator(
	void (*enumerator)(nyan_audio_output_cb cb, void *data));

// Current audio monitoring output (the endpoint glasses audio is routed to).
// Fills name and id; both empty when no provider is installed or the host is on
// its default device.
void nyan_audio_monitor_get(std::string &name, std::string &id);
void nyan_set_audio_monitor_getter(
	void (*getter)(std::string &name, std::string &id));

// Routes audio monitoring to the device. False if it could not be set (or no
// provider is installed).
bool nyan_audio_monitor_set(const std::string &name, const std::string &id);
void nyan_set_audio_monitor_setter(
	bool (*setter)(const std::string &name, const std::string &id));

// Rebuilds audio monitoring against the current device (no-op without provider).
void nyan_audio_monitor_reset();
void nyan_set_audio_monitor_resetter(void (*resetter)());

// Shows the virtual screen fullscreen on the given screen index (the glasses
// display). OBS opens a source projector; the standalone moves its own
// fullscreen window. Returns false if it could not be shown (e.g. no source
// exists yet, or no provider). log enables host-side diagnostics (the periodic
// auto-open passes false to stay quiet).
bool nyan_open_glasses_output(int monitor_index, bool log);
void nyan_set_glasses_output_opener(bool (*opener)(int monitor_index, bool log));

// Snapshots the glasses-output windows currently on the given screen so they can
// be torn down later (Windows relocates a removed display's windows onto other
// monitors, so we close them on disconnect rather than by screen). Called while
// the glasses display is present. No-op without a provider.
void nyan_track_glasses_output(int monitor_index);
void nyan_set_glasses_output_tracker(void (*tracker)(int monitor_index));

// Closes the glasses output the host last tracked/opened (called when the
// glasses display disappears). No-op without a provider.
void nyan_close_glasses_output();
void nyan_set_glasses_output_closer(void (*closer)());
