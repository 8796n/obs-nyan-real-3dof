// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Qt renders plain-text tooltips as a single unwrapped line, so long locale
// strings run far past the screen edge. Wrapping the text in <qt> markup
// turns it into rich text, which Qt word-wraps at a sane width (CJK-aware).
// Every tooltip and property long-description goes through here; the policy
// lives in CONTRIBUTING.md ("UI 文言とツールチップ").
#pragma once

#include <string>

#include "nyan_host.h"

inline std::string wrapped_tooltip(const char *locale_key)
{
	std::string s = "<qt>";
	s += nyan_text(locale_key);
	s += "</qt>";
	return s;
}
