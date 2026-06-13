// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "nyan_log.h"

namespace {
void (*g_sink)(int, const char *, va_list) = nullptr;
}

void nyan_set_log_sink(void (*sink)(int, const char *, va_list))
{
	g_sink = sink;
}

void nyan_log(int level, const char *fmt, ...)
{
	if (!g_sink)
		return;
	va_list args;
	va_start(args, fmt);
	g_sink(level, fmt, args);
	va_end(args);
}
