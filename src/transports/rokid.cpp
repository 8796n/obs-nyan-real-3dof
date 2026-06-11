// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Rokid Max / Air: continuous 64-byte sensor packets on a vendor HID
// interface, no start command.
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

// Rokid packet types on the vendor HID interface (64-byte fixed packets).
constexpr uint8_t ROKID_PKT_SENSOR = 4;
constexpr uint8_t ROKID_PKT_COMBINED = 17;

// Pairing state for Rokid type-4 packets, which deliver accel/gyro/mag as
// separate packets that share a timestamp base.
struct rokid_decode_state {
	vec3d accel;
	bool have_accel = false;
};

// Rokid Max / Air stream fixed 64-byte packets continuously with no start
// command. Layout facts from the openly documented protocol (see the
// ar-drivers-rs project's Rokid driver): type 4 = single sensor (byte 1:
// 1=accel, 2=gyro, 3=mag; u64 microsecond timestamp at offset 9; f32[3]
// vector at offset 21), type 17 = combined accel+gyro+mag (u64 nanosecond
// timestamp at offset 1; accel @9, gyro @21, mag @33). Values are SI
// (rad/s, m/s^2) in the shared X-right / Y-up / Z-toward-wearer frame, so
// there is no axis remap and no deg-to-rad conversion. Windows prepends a
// zero report-id byte for unnumbered HID reports; skip it when present.
static bool decode_rokid_packet(const uint8_t *data, size_t len,
				rokid_decode_state &st,
				decoded_sensor_report &out)
{
	if (!data || len < 2)
		return false;
	if (data[0] == 0 &&
	    (data[1] == ROKID_PKT_SENSOR || data[1] == ROKID_PKT_COMBINED)) {
		data++;
		len--;
	}
	out = {};
	if (data[0] == ROKID_PKT_SENSOR && len >= 33) {
		const uint8_t sensor_type = data[1];
		const uint64_t ts_us = read_u64_le(data + 9);
		const double v0 = read_f32_le(data + 21);
		const double v1 = read_f32_le(data + 25);
		const double v2 = read_f32_le(data + 29);
		switch (sensor_type) {
		case 1: // accelerometer: cache until the paired gyro arrives
			st.accel = {v0, v1, v2};
			st.have_accel = true;
			return false;
		case 2: // gyroscope: emit together with the latest accel
			if (!st.have_accel)
				return false;
			out.imu.gx = static_cast<float>(v0);
			out.imu.gy = static_cast<float>(v1);
			out.imu.gz = static_cast<float>(v2);
			out.imu.ax = static_cast<float>(st.accel.x);
			out.imu.ay = static_cast<float>(st.accel.y);
			out.imu.az = static_cast<float>(st.accel.z);
			out.imu.ts_us = static_cast<uint32_t>(ts_us);
			out.has_imu = true;
			return true;
		case 3: // magnetometer
			out.mag.mx = static_cast<float>(v0);
			out.mag.my = static_cast<float>(v1);
			out.mag.mz = static_cast<float>(v2);
			out.mag.temp_c = 0.0f;
			out.mag.ts_us = static_cast<uint32_t>(ts_us);
			out.has_mag = true;
			return true;
		default:
			return false;
		}
	}
	if (data[0] == ROKID_PKT_COMBINED && len >= 45) {
		const uint64_t ts_us = read_u64_le(data + 1) / 1000;
		out.imu.ax = read_f32_le(data + 9);
		out.imu.ay = read_f32_le(data + 13);
		out.imu.az = read_f32_le(data + 17);
		out.imu.gx = read_f32_le(data + 21);
		out.imu.gy = read_f32_le(data + 25);
		out.imu.gz = read_f32_le(data + 29);
		out.imu.ts_us = static_cast<uint32_t>(ts_us);
		out.has_imu = true;
		out.mag.mx = read_f32_le(data + 33);
		out.mag.my = read_f32_le(data + 37);
		out.mag.mz = read_f32_le(data + 41);
		out.mag.temp_c = 0.0f;
		out.mag.ts_us = static_cast<uint32_t>(ts_us);
		out.has_mag = true;
		return true;
	}
	return false;
}

// Probe the HID interfaces of a rokid_hid model. The device streams without
// any start command, so accepting an interface only requires one decodable
// packet.
static HANDLE open_rokid_hid_stream(hid_interface_info &selected)
{
	for (const auto &info : enumerate_hid_interfaces()) {
		if (profile_for(info.model).transport != imu_transport::rokid_hid ||
		    is_consumer_control_hid(info))
			continue;
		HANDLE h = open_hid_path_rw(info.path);
		if (h == INVALID_HANDLE_VALUE)
			continue;

		bool ok = false;
		rokid_decode_state st;
		decoded_sensor_report decoded;
		std::vector<uint8_t> report;
		const uint64_t start = os_gettime_ns();
		while (!ok && os_gettime_ns() - start < 1200000000ULL) {
			if (!hid_read_report(h, info.input_report_len, report,
					     100))
				continue;
			if (decode_rokid_packet(report.data(), report.size(),
						st, decoded))
				ok = true;
		}
		if (ok) {
			selected = info;
			return h;
		}
		CloseHandle(h);
	}
	return INVALID_HANDLE_VALUE;
}

void run_rokid_hid_session(device_manager *f, uint32_t &seen_epoch)
{
	hid_interface_info info;
	HANDLE h = open_rokid_hid_stream(info);
	if (h == INVALID_HANDLE_VALUE) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] Rokid HID connect failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO, "[obs-nyan-real-3dof] Rokid HID connected");

	rokid_decode_state st;
	std::vector<uint8_t> report;
	uint32_t seq = 0;
	uint64_t last_rx_ns = os_gettime_ns();
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::rokid_hid)
			break;
		// There is no restart command to nudge a silent stream;
		// reconnect like the other transports.
		if (os_gettime_ns() - last_rx_ns > 3000000000ULL) {
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] Rokid HID stream "
				     "stalled (no samples for 3 s); reconnecting");
			break;
		}

		if (!hid_read_report(h, info.input_report_len, report, 250))
			continue;
		decoded_sensor_report decoded;
		if (!decode_rokid_packet(report.data(), report.size(), st,
					 decoded))
			continue;
		last_rx_ns = os_gettime_ns();
		decoded.imu.seq = seq;
		decoded.mag.seq = seq;
		seq++;
		publish_sensor_samples(f, decoded.has_imu ? &decoded.imu : nullptr,
				       decoded.has_mag ? &decoded.mag : nullptr);
		maybe_log_sensor_rate(f, rate_log, "Rokid HID");
	}

	CloseHandle(h);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] Rokid HID disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}
