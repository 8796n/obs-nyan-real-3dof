// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Phone remote: a LAN WebSocket server that serves a single-page touchpad UI
// (data/remote.html) and injects the received pointer/command messages.
// Configuration lives in g_device (dock-driven, persisted); the dock's poll
// calls remote_control_sync() to reconcile the server with it.
#pragma once

#include <string>

// Starts/stops/retargets the server to match g_device.remote_* and pushes
// state updates (screen distance) to connected clients. UI thread.
void remote_control_sync();
// Stops the server and joins its threads (module unload).
void remote_control_shutdown();
// URL the phone should open (http://<lan-ip>:<port>/?t=<token>), "" while
// the server is down or no usable LAN address exists.
std::string remote_control_url();
// Revokes the current token and restarts the server with a fresh one: every
// existing session and previously scanned QR stops working. Should the
// CSPRNG be unavailable, the server stops instead (fail secure) - the old
// token dies either way. UI thread.
void remote_control_rotate_token();
// Established remote sessions, for the dock's status display.
int remote_control_client_count();
