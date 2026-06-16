// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Single source of truth for parsing the Audio Wall "exclude apps" setting, so
// the OBS source and the standalone engine read the list identically. The host
// keeps the raw text; this turns it into the lowercased exe-name tokens both
// backends match captured processes against.
#pragma once

#include <string>
#include <vector>

// Split a comma/semicolon-separated list of exe names, stripping whitespace and
// lowercasing each token (empty tokens dropped). e.g. "Discord.exe, slack.exe"
// -> {"discord.exe", "slack.exe"}.
std::vector<std::string> parse_audio_exclude_list(const std::string &raw);
