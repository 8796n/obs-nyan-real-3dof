// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Host-provided service hooks for core code that needs localization and audio
// endpoint enumeration without depending on OBS. The host (OBS plugin or
// standalone app) installs providers at startup; core code calls the wrappers.
#pragma once

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
