// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "audio_exclude.h"

#include <cctype>

std::vector<std::string> parse_audio_exclude_list(const std::string &raw)
{
	std::vector<std::string> out;
	std::string item;
	for (size_t i = 0;; ++i) {
		const char c = i < raw.size() ? raw[i] : '\0';
		if (c && c != ',' && c != ';') {
			if (!std::isspace(static_cast<unsigned char>(c)))
				item += static_cast<char>(std::tolower(
					static_cast<unsigned char>(c)));
			continue;
		}
		if (!item.empty()) {
			out.push_back(item);
			item.clear();
		}
		if (!c)
			break;
	}
	return out;
}
