// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Qt renders plain-text tooltips as a single unwrapped line, so long locale
// strings run far past the screen edge. Wrapping the text in <qt> markup
// turns it into rich text, which Qt word-wraps at a sane width (CJK-aware).
// Every tooltip and property long-description goes through here; the policy
// lives in CONTRIBUTING.md ("UI 文言とツールチップ").
#pragma once

#include <obs-module.h>

#include <string>

inline std::string wrapped_tooltip(const char *locale_key)
{
	std::string s = "<qt>";
	s += obs_module_text(locale_key);
	s += "</qt>";
	return s;
}
