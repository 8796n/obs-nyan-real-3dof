// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "nyan_host.h"

namespace {
const char *(*g_text)(const char *) = nullptr;
const char *(*g_locale)() = nullptr;
void (*g_audio_enum)(nyan_audio_output_cb, void *) = nullptr;
}

void nyan_set_text_provider(const char *(*provider)(const char *))
{
	g_text = provider;
}

const char *nyan_text(const char *key)
{
	return g_text ? g_text(key) : key;
}

void nyan_set_locale_provider(const char *(*provider)())
{
	g_locale = provider;
}

const char *nyan_locale()
{
	return g_locale ? g_locale() : "en-US";
}

void nyan_set_audio_output_enumerator(
	void (*enumerator)(nyan_audio_output_cb, void *))
{
	g_audio_enum = enumerator;
}

void nyan_enum_audio_outputs(nyan_audio_output_cb cb, void *data)
{
	if (g_audio_enum)
		g_audio_enum(cb, data);
}
