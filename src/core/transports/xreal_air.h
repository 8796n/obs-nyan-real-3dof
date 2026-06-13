// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// XREAL Air HID packet builder, shared with the One-family TCP session,
// which sends the same START IMU command over HID to wake the bridge.
#pragma once

#include <cstdint>
#include <vector>

#include "hid_io.h"

constexpr uint8_t AIR_MSG_START_IMU_DATA = 0x19;
constexpr uint8_t AIR_MSG_GET_STATIC_ID = 0x1A;

std::vector<uint8_t> build_air_packet(uint8_t msg_id,
				      const std::vector<uint8_t> &payload = {});
bool air_send_packet(HANDLE h, const hid_interface_info &info, uint8_t msg_id,
		     const std::vector<uint8_t> &payload = {});

// Control-channel framing shared across the Air (64-byte reports, MI_04
// display modes) and One (1024-byte reports, Eye camera / USB config)
// families: [0]=0xFD, [1..4]=CRC32 LE over bytes [5, 5+payload_len),
// [5..6]=payload_len u16 LE (= 17 + data length), [15..16]=msgId u16 LE,
// [22..]=data. Responses mirror the layout with a status byte at [22] and
// raw data from [23].
std::vector<uint8_t> build_xreal_control_packet(uint16_t msg_id,
						const std::vector<uint8_t> &data,
						size_t report_len);
bool xreal_control_exchange(HANDLE h, const hid_interface_info &info,
			    uint16_t msg_id, const std::vector<uint8_t> &data,
			    uint8_t *out_status, std::vector<uint8_t> *out_data);

// Standard IEEE 802.3 CRC32, shared by the Air packet framing and the Nreal
// Light MCU ASCII protocol.
uint32_t air_crc32(const uint8_t *data, size_t len);
