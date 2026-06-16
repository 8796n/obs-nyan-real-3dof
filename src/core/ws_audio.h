// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Browser-extension audio ingest wire protocol (tools/chrome-extension over the
// WebSocket loopback). The OBS source and the standalone Audio Wall share this
// decode so the wire format + version live in exactly one place (they only
// differ in the sink: an obs_source vs a WASAPI mixer). See CONTRIBUTING.md.
#pragma once

#include <cstdint>
#include <string>

#include "nyan_json.h"

// Bump in lockstep with the extension; both backends compare against this so a
// version mismatch is detectable in one place.
constexpr long long WS_AUDIO_PROTOCOL_VERSION = 1;

// A decoded text control message. norm_x is raw (callers clamp to -0.5..0.5).
struct ws_audio_msg {
	bool is_meta = false;  // type == "meta"
	bool is_close = false; // type == "close"
	uint32_t stream = 0;   // per-connection stream id
	long long proto = 0;   // "v" (sender's protocol version)
	float norm_x = 0.0f;   // tab x on the virtual desktop
	std::string label;     // human label (UI)
	uint32_t sample_rate = 48000;
	int channels = 2; // 1 or 2
	std::string exe;  // sender exe name (as sent)
};
ws_audio_msg ws_audio_parse_msg(const nyan_json &msg);

// Decode a binary PCM frame: 4-byte little-endian stream id followed by
// interleaved int16 samples. False when shorter than the 4-byte header; else
// fills stream_id, the PCM pointer and its byte length (frames = pcm_bytes /
// (2 * channels), channels taken from the stream's prior meta).
bool ws_audio_parse_frame(const uint8_t *data, size_t len, uint32_t &stream_id,
			  const int16_t *&pcm, size_t &pcm_bytes);
