// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "nyan_host.h"

namespace {
const char *(*g_text)(const char *) = nullptr;
const char *(*g_locale)() = nullptr;
void (*g_audio_enum)(nyan_audio_output_cb, void *) = nullptr;
void (*g_audio_mon_get)(std::string &, std::string &) = nullptr;
bool (*g_audio_mon_set)(const std::string &, const std::string &) = nullptr;
void (*g_audio_mon_reset)() = nullptr;
bool (*g_open_glasses_output)(int, bool) = nullptr;
void (*g_track_glasses_output)(int) = nullptr;
void (*g_close_glasses_output)() = nullptr;
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

void nyan_set_audio_monitor_getter(
	void (*getter)(std::string &name, std::string &id))
{
	g_audio_mon_get = getter;
}

void nyan_audio_monitor_get(std::string &name, std::string &id)
{
	name.clear();
	id.clear();
	if (g_audio_mon_get)
		g_audio_mon_get(name, id);
}

void nyan_set_audio_monitor_setter(
	bool (*setter)(const std::string &name, const std::string &id))
{
	g_audio_mon_set = setter;
}

bool nyan_audio_monitor_set(const std::string &name, const std::string &id)
{
	return g_audio_mon_set ? g_audio_mon_set(name, id) : false;
}

void nyan_set_audio_monitor_resetter(void (*resetter)())
{
	g_audio_mon_reset = resetter;
}

void nyan_audio_monitor_reset()
{
	if (g_audio_mon_reset)
		g_audio_mon_reset();
}

void nyan_set_glasses_output_opener(bool (*opener)(int monitor_index, bool log))
{
	g_open_glasses_output = opener;
}

bool nyan_open_glasses_output(int monitor_index, bool log)
{
	return g_open_glasses_output ? g_open_glasses_output(monitor_index, log)
				     : false;
}

void nyan_set_glasses_output_tracker(void (*tracker)(int monitor_index))
{
	g_track_glasses_output = tracker;
}

void nyan_track_glasses_output(int monitor_index)
{
	if (g_track_glasses_output)
		g_track_glasses_output(monitor_index);
}

void nyan_set_glasses_output_closer(void (*closer)())
{
	g_close_glasses_output = closer;
}

void nyan_close_glasses_output()
{
	if (g_close_glasses_output)
		g_close_glasses_output();
}
