// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "process_util.h"

#include <windows.h>

#include <cctype>

std::string pid_exe_lower(unsigned long pid)
{
	if (!pid)
		return "";
	HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
	if (!h)
		return "";
	wchar_t path[MAX_PATH];
	DWORD len = MAX_PATH;
	std::string out;
	if (QueryFullProcessImageNameW(h, 0, path, &len)) {
		const wchar_t *base = wcsrchr(path, L'\\');
		const wchar_t *name = base ? base + 1 : path;
		const int n = WideCharToMultiByte(CP_UTF8, 0, name, -1, nullptr,
						  0, nullptr, nullptr);
		if (n > 1) {
			out.resize(static_cast<size_t>(n - 1));
			WideCharToMultiByte(CP_UTF8, 0, name, -1, out.data(), n,
					    nullptr, nullptr);
		}
		for (char &c : out)
			c = static_cast<char>(
				std::tolower(static_cast<unsigned char>(c)));
	}
	CloseHandle(h);
	return out;
}
