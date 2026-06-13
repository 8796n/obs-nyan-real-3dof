// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Core monotonic clock. Replaces libobs os_gettime_ns() in OBS-independent
// code. steady_clock has a different epoch than os_gettime_ns(), so the
// value is only meaningful for deltas - which is all core code uses it for.
#pragma once

#include <chrono>
#include <cstdint>

inline uint64_t nyan_now_ns()
{
	return static_cast<uint64_t>(
		std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::steady_clock::now().time_since_epoch())
			.count());
}
