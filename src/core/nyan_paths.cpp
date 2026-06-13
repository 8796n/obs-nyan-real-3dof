// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "nyan_paths.h"

namespace {
std::string (*g_config)(const char *) = nullptr;
std::string (*g_asset)(const char *) = nullptr;
}

void nyan_set_path_resolvers(std::string (*config)(const char *),
			     std::string (*asset)(const char *))
{
	g_config = config;
	g_asset = asset;
}

std::string nyan_config_path(const char *filename)
{
	return g_config ? g_config(filename) : std::string();
}

std::string nyan_asset_path(const char *filename)
{
	return g_asset ? g_asset(filename) : std::string();
}
