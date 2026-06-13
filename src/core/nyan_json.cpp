// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "nyan_json.h"

#include <fstream>
#include <iterator>
#include <sstream>

nyan_json nyan_json_parse(const std::string &text)
{
	// allow_exceptions = false: malformed input yields a discarded value
	// instead of throwing, matching the rest of core's no-throw style.
	return nyan_json::parse(text, nullptr, /*allow_exceptions=*/false);
}

nyan_json nyan_json_parse_file(const std::string &path)
{
	std::ifstream f(path, std::ios::binary);
	if (!f)
		return nyan_json(nyan_json::value_t::discarded);
	std::ostringstream ss;
	ss << f.rdbuf();
	return nyan_json_parse(ss.str());
}

namespace {
const nyan_json *find_member(const nyan_json &o, const char *key)
{
	if (!o.is_object())
		return nullptr;
	const auto it = o.find(key);
	return it == o.end() ? nullptr : &*it;
}
} // namespace

bool nyan_json_get_bool(const nyan_json &o, const char *key, bool def)
{
	const nyan_json *v = find_member(o, key);
	return (v && v->is_boolean()) ? v->get<bool>() : def;
}

double nyan_json_get_double(const nyan_json &o, const char *key, double def)
{
	const nyan_json *v = find_member(o, key);
	return (v && v->is_number()) ? v->get<double>() : def;
}

long long nyan_json_get_int(const nyan_json &o, const char *key, long long def)
{
	const nyan_json *v = find_member(o, key);
	if (!v || !v->is_number())
		return def;
	if (v->is_number_integer())
		return v->get<long long>();
	return static_cast<long long>(v->get<double>());
}

std::string nyan_json_get_string(const nyan_json &o, const char *key,
				 const std::string &def)
{
	const nyan_json *v = find_member(o, key);
	return (v && v->is_string()) ? v->get<std::string>() : def;
}
