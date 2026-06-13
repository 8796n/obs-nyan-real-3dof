// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// HID interface enumeration with registry-based model detection, plus
// overlapped HID report I/O with timeouts.
#pragma once

#include <windows.h>
#include <hidsdi.h>

#include <string>
#include <vector>

#include "nyan_types.h"

struct hid_interface_info {
	std::wstring path;
	std::wstring product;
	uint16_t vid = 0;
	uint16_t pid = 0;
	USAGE usage_page = 0;
	USAGE usage = 0;
	USHORT input_report_len = 0;
	USHORT output_report_len = 0;
	model_id model = MODEL_UNKNOWN;
};

std::vector<hid_interface_info> enumerate_hid_interfaces();
model_id detect_hid_model(std::string *out_present = nullptr);
bool is_consumer_control_hid(const hid_interface_info &info);

HANDLE open_hid_path_rw(const std::wstring &path);
bool wait_overlapped_result(HANDLE h, OVERLAPPED &ov, DWORD timeout_ms,
			    DWORD &bytes);
bool hid_write_report(HANDLE h, USHORT output_len,
		      const std::vector<uint8_t> &payload, DWORD timeout_ms);
bool hid_read_report(HANDLE h, USHORT input_len, std::vector<uint8_t> &data,
		     DWORD timeout_ms);
