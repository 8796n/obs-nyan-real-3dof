// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Minimal localhost WebSocket server for the Audio Wall's browser-extension
// ingest: text frames carry JSON control messages, binary frames carry
// multiplexed PCM. Hand-rolled (RFC 6455 server side only), no dependencies
// beyond WinSock and the Windows crypto API for the handshake SHA-1.
#pragma once

#include <cstdint>
#include <functional>

typedef struct obs_data obs_data_t;

struct ws_server_callbacks {
	// Parsed JSON of a text frame. The obs_data is released by the server
	// after the callback returns.
	std::function<void(uint64_t conn, obs_data_t *msg)> on_text;
	std::function<void(uint64_t conn, const uint8_t *data, size_t len)>
		on_binary;
	// Connection ended (graceful or not); always called once per conn.
	std::function<void(uint64_t conn)> on_close;
};

class ws_audio_server {
public:
	~ws_audio_server();
	// Binds 127.0.0.1:port. Callbacks run on per-connection threads.
	bool start(uint16_t port, ws_server_callbacks cb);
	// Stops accepting, closes all connections, joins all threads. After
	// stop() returns no callback is running or will run.
	void stop();
	bool running() const { return running_; }
	uint16_t port() const { return port_; }

private:
	struct impl;
	impl *impl_ = nullptr;
	bool running_ = false;
	uint16_t port_ = 0;
};
