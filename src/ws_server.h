// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Minimal WebSocket server shared by the Audio Wall's browser-extension
// ingest (loopback) and the phone remote (LAN): text frames carry JSON
// control messages, binary frames carry multiplexed PCM. Non-upgrade GET
// requests can serve a static page (the remote's touchpad UI). Hand-rolled
// (RFC 6455 server side only), no dependencies beyond WinSock and the
// Windows crypto API for the handshake SHA-1.
#pragma once

#include <cstdint>
#include <functional>
#include <string>

typedef struct obs_data obs_data_t;

struct ws_server_callbacks {
	// Parsed JSON of a text frame. The obs_data is released by the server
	// after the callback returns.
	std::function<void(uint64_t conn, obs_data_t *msg)> on_text;
	std::function<void(uint64_t conn, const uint8_t *data, size_t len)>
		on_binary;
	// WebSocket handshake completed; send_text is valid from here on.
	std::function<void(uint64_t conn)> on_open;
	// Connection ended (graceful or not); always called once per conn
	// that reached on_open.
	std::function<void(uint64_t conn)> on_close;
	// Plain (non-upgrade) GET: return the response body for the request
	// target ("/", "/index.html?x=y", ...), empty string = 404. Unset =
	// non-upgrade requests are rejected with 400 as before.
	std::function<std::string(const std::string &target)> on_http_get;
	// Gate a WebSocket upgrade by its request target (path + query, e.g.
	// "/ws?t=TOKEN"); false sends 403. Unset = accept all upgrades.
	std::function<bool(const std::string &target)> on_ws_accept;
	// Gate every request (GET and upgrade) by its Host header; false sends
	// 403 before any other handling. Used by the LAN remote to require an
	// IP-literal Host (a DNS-rebinding defense). Unset = accept any Host.
	std::function<bool(const std::string &host)> on_check_host;
};

class ws_server {
public:
	~ws_server();
	// Binds 127.0.0.1:port, or 0.0.0.0:port with lan = true (exposes the
	// server to the local network - gate it with on_ws_accept). Callbacks
	// run on per-connection threads.
	bool start(uint16_t port, ws_server_callbacks cb, bool lan = false);
	// Stops accepting, closes all connections, joins all threads. After
	// stop() returns no callback is running or will run.
	void stop();
	bool running() const { return running_; }
	uint16_t port() const { return port_; }
	// Sends a text frame to one established connection from any thread;
	// false when the connection is gone. Safe against the connection's
	// own control-frame sends.
	bool send_text(uint64_t conn, const std::string &text);
	// Asks one connection to tear down (server-side idle eviction): shuts
	// the socket down so the frame loop unblocks and runs the normal
	// teardown (on_close fires from the connection's own thread).
	void close_conn(uint64_t conn);

private:
	struct impl;
	impl *impl_ = nullptr;
	bool running_ = false;
	uint16_t port_ = 0;
};
