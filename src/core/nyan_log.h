// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Core logging hook. The core library never depends on OBS; the host
// (OBS plugin or standalone app) installs a sink that actually emits.
// Levels mirror libobs LOG_* values so an OBS host can pass them straight
// to blogva (a static_assert in the host verifies the mapping).
#pragma once

#include <cstdarg>

enum {
	NYAN_LOG_DEBUG = 100,
	NYAN_LOG_INFO = 200,
	NYAN_LOG_WARNING = 300,
	NYAN_LOG_ERROR = 400,
};

// printf-style log. Routed to the installed sink; a no-op until one is set.
void nyan_log(int level, const char *fmt, ...);

// Install the backend that emits log lines (OBS host: forwards to blogva).
// Pass nullptr to mute. Not thread-safe with concurrent nyan_log calls;
// install once at startup before worker threads run.
void nyan_set_log_sink(void (*sink)(int level, const char *fmt, va_list args));
