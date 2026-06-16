// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// OBS-free Win32 process helpers shared by the OBS Audio Wall source and the
// standalone engine, so the "which app is this PID" lookup lives in one place.
#pragma once

#include <string>

// Lowercase exe basename of a process ("discord.exe"); "" when unavailable.
// pid is a Win32 DWORD (unsigned long); kept windows.h-free in the header.
std::string pid_exe_lower(unsigned long pid);
