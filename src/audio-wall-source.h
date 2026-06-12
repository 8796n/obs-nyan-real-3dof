// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Audio Wall engine: auto-captures the audio of apps on screen and mixes
// them with head-tracked spatial panning. Hosted by the Display Wall source
// as its checkable "spatial audio" property group (key "audio_wall") rather
// than being a source of its own; the host forwards its source callbacks.
#pragma once

#include <obs-module.h>

struct audio_wall_engine;

// Registers the hidden push-audio sink the engine's browser-extension
// streams use (CAP_DISABLED). Call once from obs_module_load.
void register_nyan_real_ws_audio_source();

// parent is the hosting source; the engine keeps its private capture
// children active through it. The caller applies settings with
// audio_wall_update right after creating.
audio_wall_engine *audio_wall_create(obs_source_t *parent);
void audio_wall_destroy(audio_wall_engine *engine);
void audio_wall_update(audio_wall_engine *engine, obs_data_t *settings);
void audio_wall_defaults(obs_data_t *settings);
// Appends the checkable "audio_wall" group to the host's properties.
// engine may be null (group unchecked); the capture list then stays empty.
void audio_wall_add_properties(audio_wall_engine *engine,
			       obs_properties_t *props);
void audio_wall_tick(audio_wall_engine *engine);
void audio_wall_show(audio_wall_engine *engine);
void audio_wall_hide(audio_wall_engine *engine);
void audio_wall_enum_active(audio_wall_engine *engine,
			    obs_source_enum_proc_t enum_callback, void *param);
