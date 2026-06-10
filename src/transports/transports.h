// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Per-family IMU session loops. Each call runs one connection lifetime:
// open, stream, publish into the tracker, and return on disconnect, stop or
// model change so the worker can reconnect or switch transports.
#pragma once

#include <cstdint>

struct device_manager;

void run_one_bridge_tcp_session(device_manager *f, uint32_t &seen_epoch,
				uint64_t &last_detect_ns);
void run_air_hid_session(device_manager *f, uint32_t &seen_epoch,
			 uint64_t &last_detect_ns);
void run_rayneo_hid_session(device_manager *f, uint32_t &seen_epoch,
			    uint64_t &last_detect_ns);
void run_sensor_api_session(device_manager *f, uint32_t &seen_epoch,
			    uint64_t &last_detect_ns);
void run_rokid_hid_session(device_manager *f, uint32_t &seen_epoch,
			   uint64_t &last_detect_ns);
void run_viture_hid_session(device_manager *f, uint32_t &seen_epoch,
			    uint64_t &last_detect_ns);
