// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "ws_audio.h"

ws_audio_msg ws_audio_parse_msg(const nyan_json &msg)
{
	ws_audio_msg m;
	const std::string type = nyan_json_get_string(msg, "type");
	m.is_close = type == "close";
	m.is_meta = type == "meta";
	m.stream = static_cast<uint32_t>(nyan_json_get_int(msg, "stream") &
					 0xFFFFFFFFLL);
	m.proto = nyan_json_get_int(msg, "v");
	m.norm_x = static_cast<float>(nyan_json_get_double(msg, "norm_x"));
	m.label = nyan_json_get_string(msg, "label");
	const long long sr = nyan_json_get_int(msg, "sample_rate");
	if (sr >= 8000 && sr <= 192000)
		m.sample_rate = static_cast<uint32_t>(sr);
	m.channels = nyan_json_get_int(msg, "channels") == 1 ? 1 : 2;
	m.exe = nyan_json_get_string(msg, "exe");
	return m;
}

bool ws_audio_parse_frame(const uint8_t *data, size_t len, uint32_t &stream_id,
			  const int16_t *&pcm, size_t &pcm_bytes)
{
	if (len < 4)
		return false;
	stream_id = static_cast<uint32_t>(data[0]) |
		    (static_cast<uint32_t>(data[1]) << 8) |
		    (static_cast<uint32_t>(data[2]) << 16) |
		    (static_cast<uint32_t>(data[3]) << 24);
	pcm = reinterpret_cast<const int16_t *>(data + 4);
	pcm_bytes = len - 4;
	return true;
}
