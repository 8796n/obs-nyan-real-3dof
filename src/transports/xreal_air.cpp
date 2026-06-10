// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// XREAL Air family: IMU/MAG over HID input reports after a START command.
#include <obs-module.h>

#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include "../device_manager.h"
#include "../device_registry.h"
#include "../hid_io.h"
#include "../math_util.h"
#include "transports.h"
#include "xreal_air.h"

static int16_t read_air_mag_i16(const uint8_t *p)
{
	const uint16_t u = static_cast<uint16_t>(p[0]) |
			   static_cast<uint16_t>((p[1] ^ 0x80) << 8);
	return static_cast<int16_t>(u);
}

struct air_timestamp_state {
	bool have_last = false;
	uint64_t last_sensor_ts = 0;
	uint32_t ts_us = 0;
};

static uint32_t air_ts_us32_from_report(uint64_t ts64, air_timestamp_state &state)
{
	if (!state.have_last) {
		state.have_last = true;
		state.last_sensor_ts = ts64;
		state.ts_us = now_us32();
		return state.ts_us;
	}

	const uint64_t delta = ts64 >= state.last_sensor_ts ? ts64 - state.last_sensor_ts : 0;
	state.last_sensor_ts = ts64;
	const uint64_t delta_us = delta / 1000ULL;
	if (delta_us > 500000ULL) {
		state.ts_us = now_us32();
		return state.ts_us;
	}
	state.ts_us += static_cast<uint32_t>(delta_us);
	return state.ts_us;
}

static bool decode_air_report(const uint8_t *data, size_t len,
			      air_timestamp_state &ts_state,
			      decoded_sensor_report &out)
{
	if (!data || len < 64)
		return false;
	const uint16_t signature = read_u16_le(data);
	if (signature == 0x53AA)
		return false;
	if (signature != 0x0201)
		return false;

	const uint32_t ts_us = air_ts_us32_from_report(read_u64_le(data + 4), ts_state);
	const int16_t gyro_mul = read_i16_le(data + 12);
	const int32_t gyro_div = read_i32_le(data + 14);
	const int16_t acc_mul = read_i16_le(data + 27);
	const int32_t acc_div = read_i32_le(data + 29);
	if (gyro_div == 0 || acc_div == 0)
		return false;

	const double deg_to_rad = PI / 180.0;
	const double gx0 =
		(static_cast<double>(read_i24_le(data + 18)) * gyro_mul / gyro_div) *
		deg_to_rad;
	const double gy0 =
		(static_cast<double>(read_i24_le(data + 21)) * gyro_mul / gyro_div) *
		deg_to_rad;
	const double gz0 =
		(static_cast<double>(read_i24_le(data + 24)) * gyro_mul / gyro_div) *
		deg_to_rad;
	const double ax0 = static_cast<double>(read_i24_le(data + 33)) * acc_mul /
			   acc_div;
	const double ay0 = static_cast<double>(read_i24_le(data + 36)) * acc_mul /
			   acc_div;
	const double az0 = static_cast<double>(read_i24_le(data + 39)) * acc_mul /
			   acc_div;

	const double t_gx = gy0;
	const double t_gy = gx0;
	const double t_gz = gz0;
	const double t_ax = ay0;
	const double t_ay = ax0;
	const double t_az = az0;

	out.imu.ts_us = ts_us;
	out.imu.gx = static_cast<float>(-t_gy);
	out.imu.gy = static_cast<float>(+t_gz);
	out.imu.gz = static_cast<float>(+t_gx);
	out.imu.ax = static_cast<float>(-t_ay);
	out.imu.ay = static_cast<float>(+t_az);
	out.imu.az = static_cast<float>(+t_ax);
	out.has_imu = true;

	const int16_t mag_mul = read_i16_be(data + 42);
	const int32_t mag_div = read_i32_be(data + 44);
	if (mag_div != 0) {
		const double mx0 =
			static_cast<double>(read_air_mag_i16(data + 48)) * mag_mul /
			mag_div;
		const double my0 =
			static_cast<double>(read_air_mag_i16(data + 50)) * mag_mul /
			mag_div;
		const double mz0 =
			static_cast<double>(read_air_mag_i16(data + 52)) * mag_mul /
			mag_div;
		const double t_mx = my0;
		const double t_my = mx0;
		const double t_mz = mz0;
		out.mag.ts_us = ts_us;
		out.mag.mx = static_cast<float>(+t_my);
		out.mag.my = static_cast<float>(-t_mz);
		out.mag.mz = static_cast<float>(-t_mx);
		out.mag.temp_c = 0.0f;
		out.has_mag = true;
	}

	return out.has_imu || out.has_mag;
}

static uint32_t air_crc32(const uint8_t *data, size_t len)
{
	static uint32_t table[256] = {};
	static bool table_ready = false;
	if (!table_ready) {
		for (uint32_t i = 0; i < 256; ++i) {
			uint32_t c = i;
			for (int k = 0; k < 8; ++k)
				c = (c & 1) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
			table[i] = c;
		}
		table_ready = true;
	}

	uint32_t c = 0xFFFFFFFFU;
	for (size_t i = 0; i < len; ++i)
		c = table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
	return c ^ 0xFFFFFFFFU;
}

std::vector<uint8_t> build_air_packet(uint8_t msg_id,
				      const std::vector<uint8_t> &payload)
{
	const uint16_t packet_len = static_cast<uint16_t>(3 + payload.size());
	std::vector<uint8_t> chk(3 + payload.size());
	chk[0] = static_cast<uint8_t>(packet_len & 0xFF);
	chk[1] = static_cast<uint8_t>((packet_len >> 8) & 0xFF);
	chk[2] = msg_id;
	if (!payload.empty())
		std::memcpy(chk.data() + 3, payload.data(), payload.size());

	const uint32_t checksum = air_crc32(chk.data(), chk.size());
	std::vector<uint8_t> packet(1 + 4 + chk.size());
	packet[0] = 0xAA;
	packet[1] = static_cast<uint8_t>(checksum & 0xFF);
	packet[2] = static_cast<uint8_t>((checksum >> 8) & 0xFF);
	packet[3] = static_cast<uint8_t>((checksum >> 16) & 0xFF);
	packet[4] = static_cast<uint8_t>((checksum >> 24) & 0xFF);
	std::memcpy(packet.data() + 5, chk.data(), chk.size());
	return packet;
}

bool air_send_packet(HANDLE h, const hid_interface_info &info, uint8_t msg_id,
		     const std::vector<uint8_t> &payload)
{
	return hid_write_report(h, info.output_report_len,
				build_air_packet(msg_id, payload), 250);
}

static bool looks_like_air_report(const std::vector<uint8_t> &data)
{
	if (data.empty())
		return false;
	if (data[0] == 0xAA)
		return true;
	if (data.size() >= 2) {
		const uint16_t sig = read_u16_le(data.data());
		return sig == 0x0201 || sig == 0x53AA;
	}
	return false;
}

static HANDLE open_air_hid_stream(hid_interface_info &selected)
{
	for (const auto &info : enumerate_hid_interfaces()) {
		if (profile_for(info.model).transport != imu_transport::air_hid ||
		    is_consumer_control_hid(info))
			continue;
		HANDLE h = open_hid_path_rw(info.path);
		if (h == INVALID_HANDLE_VALUE)
			continue;

		bool ok = false;
		air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x00});
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		air_send_packet(h, info, AIR_MSG_GET_STATIC_ID);
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x01});

		const uint64_t start = os_gettime_ns();
		while (os_gettime_ns() - start < 800000000ULL) {
			std::vector<uint8_t> report;
			if (hid_read_report(h, info.input_report_len, report, 100) &&
			    looks_like_air_report(report)) {
				ok = true;
				break;
			}
		}

		if (ok) {
			selected = info;
			return h;
		}
		air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x00});
		CloseHandle(h);
	}
	return INVALID_HANDLE_VALUE;
}

void run_air_hid_session(device_manager *f, uint32_t &seen_epoch,
				uint64_t &last_detect_ns)
{
	hid_interface_info info;
	HANDLE h = open_air_hid_stream(info);
	if (h == INVALID_HANDLE_VALUE) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] Air HID connect failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO, "[obs-nyan-real-3dof] Air HID connected");

	air_timestamp_state ts_state;
	uint32_t seq = 0;
	uint64_t last_rx_ns = os_gettime_ns();
	uint64_t last_start_retry_ns = last_rx_ns;
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		refresh_detected_model(f, last_detect_ns);
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::air_hid)
			break;

		std::vector<uint8_t> report;
		if (!hid_read_report(h, info.input_report_len, report, 250)) {
			const uint64_t now = os_gettime_ns();
			if (now - last_rx_ns > 1200000000ULL &&
			    now - last_start_retry_ns > 1200000000ULL) {
				last_start_retry_ns = now;
				air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x00});
				std::this_thread::sleep_for(std::chrono::milliseconds(80));
				air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x01});
			}
			continue;
		}
		decoded_sensor_report decoded;
		if (!decode_air_report(report.data(), report.size(), ts_state, decoded))
			continue;
		last_rx_ns = os_gettime_ns();
		decoded.imu.seq = seq;
		decoded.mag.seq = seq;
		seq++;
		publish_sensor_samples(f, decoded.has_imu ? &decoded.imu : nullptr,
				       decoded.has_mag ? &decoded.mag : nullptr);
		maybe_log_sensor_rate(f, rate_log, "Air HID");
	}

	air_send_packet(h, info, AIR_MSG_START_IMU_DATA, {0x00});
	CloseHandle(h);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] Air HID disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}
