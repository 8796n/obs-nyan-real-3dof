// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Core JSON type and helpers. Wraps the vendored nlohmann/json (MIT, see
// vendor/nlohmann/json.hpp) behind a stable alias so consumers write
// `nyan_json` and the backing library can be swapped without touching them.
// This is the OBS-independent settings/config/remote-protocol representation;
// the OBS host converts obs_data_t <-> nyan_json at its boundary.
#pragma once

#include <string>

#include "vendor/nlohmann/json.hpp"

using nyan_json = nlohmann::json;

// Parse text into a JSON value. Never throws; returns a discarded value
// (nyan_json::value_t::discarded, i.e. is_discarded() == true) on error.
nyan_json nyan_json_parse(const std::string &text);

// Read and parse a UTF-8 JSON file. Returns a discarded value if the file is
// missing or malformed (the caller treats a missing file as "nothing to add").
nyan_json nyan_json_parse_file(const std::string &path);

// Lenient object-field readers mirroring obs_data_get_*'s tolerance: a missing
// key or wrong type yields the default, and integers and doubles interconvert.
// These never throw, so they are safe on the remote-protocol and Audio Wall
// ingest messages whose wire types come from a browser/extension.
bool nyan_json_get_bool(const nyan_json &o, const char *key, bool def = false);
double nyan_json_get_double(const nyan_json &o, const char *key,
			    double def = 0.0);
long long nyan_json_get_int(const nyan_json &o, const char *key,
			    long long def = 0);
std::string nyan_json_get_string(const nyan_json &o, const char *key,
				 const std::string &def = std::string());
