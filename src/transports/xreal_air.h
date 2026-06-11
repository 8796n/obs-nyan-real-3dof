// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// XREAL Air HID packet builder, shared with the One-family TCP session,
// which sends the same START IMU command over HID to wake the bridge.
#pragma once

#include <cstdint>
#include <vector>

#include "../hid_io.h"

constexpr uint8_t AIR_MSG_START_IMU_DATA = 0x19;
constexpr uint8_t AIR_MSG_GET_STATIC_ID = 0x1A;

std::vector<uint8_t> build_air_packet(uint8_t msg_id,
				      const std::vector<uint8_t> &payload = {});
bool air_send_packet(HANDLE h, const hid_interface_info &info, uint8_t msg_id,
		     const std::vector<uint8_t> &payload = {});

// Standard IEEE 802.3 CRC32, shared by the Air packet framing and the Nreal
// Light MCU ASCII protocol.
uint32_t air_crc32(const uint8_t *data, size_t len);
