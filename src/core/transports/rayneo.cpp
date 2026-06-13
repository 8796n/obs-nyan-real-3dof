// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// RayNeo Air family: [0x66, cmd, arg] commands and [0x99, kind, len, ...]
// framed IMU/MAG over HID, mirroring xrealonenet/3dof/js/rayneo_driver.js.
#include "nyan_log.h"

#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include "nyan_time.h"

#include "device_manager.h"
#include "device_registry.h"
#include "hid_io.h"
#include "math_util.h"
#include "transports.h"

// RayNeo HID protocol, mirroring xrealonenet/3dof/js/rayneo_driver.js.
// Commands go out as a 64-byte report (id 0): [0x66, cmd, arg, 0...]. Input
// reports carry a byte stream of [0x99, kind, len, ...] frames.
constexpr uint8_t RAYNEO_CMD_HEADER = 0x66;
constexpr uint8_t RAYNEO_CMD_ACQUIRE_DEVICE_INFO = 0x00;
constexpr uint8_t RAYNEO_CMD_OPEN_IMU = 0x01;
constexpr uint8_t RAYNEO_CMD_CLOSE_IMU = 0x02;
constexpr uint8_t RAYNEO_PKT_MAGIC = 0x99;
constexpr uint8_t RAYNEO_PKT_SENSOR = 0x65;
constexpr uint8_t RAYNEO_PKT_RESPONSE = 0xC8;

// RayNeo HID input reports carry a byte stream of [0x99, kind, len, ...]
// frames that may split across reports; scan for the magic byte and
// reassemble, mirroring the packet assembler in rayneo_driver.js. Feeding the
// raw Windows report buffer (including a leading report-id byte) is fine
// because the scan skips everything before the magic.
class rayneo_packet_assembler {
public:
	void append(const uint8_t *data, size_t len)
	{
		if (!data || len == 0)
			return;
		if (stash_.size() + len > 4096)
			stash_.clear();
		stash_.insert(stash_.end(), data, data + len);
	}

	bool next_packet(std::vector<uint8_t> &out)
	{
		size_t offset = 0;
		while (offset + 4 <= stash_.size()) {
			if (stash_[offset] != RAYNEO_PKT_MAGIC) {
				offset++;
				continue;
			}
			const size_t packet_len = stash_[offset + 2];
			if (packet_len < 4) {
				offset++;
				continue;
			}
			if (offset + packet_len > stash_.size())
				break; // partial packet; keep from the magic
			out.assign(stash_.begin() + static_cast<ptrdiff_t>(offset),
				   stash_.begin() +
					   static_cast<ptrdiff_t>(offset + packet_len));
			consume(offset + packet_len);
			return true;
		}
		consume(offset);
		return false;
	}

private:
	void consume(size_t n)
	{
		if (n == 0)
			return;
		if (n >= stash_.size())
			stash_.clear();
		else
			stash_.erase(stash_.begin(),
				     stash_.begin() + static_cast<ptrdiff_t>(n));
	}

	std::vector<uint8_t> stash_;
};

// Mirrors parseRayNeoSensor in rayneo_driver.js: float32 LE fields, gyro in
// deg/s, a 100 us tick counter at offset 40. Unlike the XREAL Air decoder the
// reference driver feeds the device axes straight to the tracker, so there is
// no axis remap here.
static bool decode_rayneo_sensor_packet(const uint8_t *pkt, size_t len,
					decoded_sensor_report &out)
{
	out = {};
	if (!pkt || len < 56 || pkt[0] != RAYNEO_PKT_MAGIC ||
	    pkt[1] != RAYNEO_PKT_SENSOR)
		return false;

	const uint32_t ts_us = read_u32_le(pkt + 40) * 100u;
	const double deg_to_rad = PI / 180.0;
	out.imu.ts_us = ts_us;
	out.imu.gx = static_cast<float>(read_f32_le(pkt + 16) * deg_to_rad);
	out.imu.gy = static_cast<float>(read_f32_le(pkt + 20) * deg_to_rad);
	out.imu.gz = static_cast<float>(read_f32_le(pkt + 24) * deg_to_rad);
	out.imu.ax = read_f32_le(pkt + 4);
	out.imu.ay = read_f32_le(pkt + 8);
	out.imu.az = read_f32_le(pkt + 12);
	out.has_imu = true;

	out.mag.ts_us = ts_us;
	out.mag.mx = read_f32_le(pkt + 32);
	out.mag.my = read_f32_le(pkt + 36);
	out.mag.mz = read_f32_le(pkt + 52);
	out.mag.temp_c = read_f32_le(pkt + 28);
	out.has_mag = true;
	return true;
}

// Mirrors parseRayNeoDeviceInfo in rayneo_driver.js: the response to
// ACQUIRE_DEVICE_INFO carries the board id, which decides the IMU mount
// offset (boardId 0x3A = -20 deg, otherwise 0 deg).
static bool decode_rayneo_device_info(const uint8_t *pkt, size_t len,
				      int &board_id)
{
	if (!pkt || len <= 43 || pkt[0] != RAYNEO_PKT_MAGIC ||
	    pkt[1] != RAYNEO_PKT_RESPONSE ||
	    pkt[8] != RAYNEO_CMD_ACQUIRE_DEVICE_INFO)
		return false;
	board_id = pkt[21];
	return true;
}

static bool rayneo_send_command(HANDLE h, const hid_interface_info &info,
				uint8_t cmd, uint8_t arg = 0)
{
	// The reference driver sends a full 64-byte report with report id 0.
	std::vector<uint8_t> payload(64, 0);
	payload[0] = RAYNEO_CMD_HEADER;
	payload[1] = cmd;
	payload[2] = arg;
	return hid_write_report(h, info.output_report_len, payload, 250);
}

// Probe every non-consumer-control HID interface of a rayneo_hid model:
// request device info, open the IMU, and accept the interface once a sensor
// packet assembles, mirroring probeDevice in rayneo_driver.js. The device-info
// response usually arrives during this probe, so the board id is captured here
// (board_id stays -1 when it was not seen).
static HANDLE open_rayneo_hid_stream(hid_interface_info &selected, int &board_id)
{
	board_id = -1;
	for (const auto &info : enumerate_hid_interfaces()) {
		if (profile_for(info.model).transport != imu_transport::rayneo_hid ||
		    is_consumer_control_hid(info))
			continue;
		HANDLE h = open_hid_path_rw(info.path);
		if (h == INVALID_HANDLE_VALUE)
			continue;

		rayneo_send_command(h, info, RAYNEO_CMD_ACQUIRE_DEVICE_INFO);
		std::this_thread::sleep_for(std::chrono::milliseconds(80));
		rayneo_send_command(h, info, RAYNEO_CMD_OPEN_IMU);

		bool ok = false;
		int probe_board_id = -1;
		rayneo_packet_assembler assembler;
		std::vector<uint8_t> report;
		std::vector<uint8_t> packet;
		decoded_sensor_report decoded;
		const uint64_t start = nyan_now_ns();
		while (!ok && nyan_now_ns() - start < 1200000000ULL) {
			if (!hid_read_report(h, info.input_report_len, report, 100))
				continue;
			assembler.append(report.data(), report.size());
			while (assembler.next_packet(packet)) {
				if (decode_rayneo_device_info(packet.data(),
							      packet.size(),
							      probe_board_id))
					continue;
				if (decode_rayneo_sensor_packet(packet.data(),
								packet.size(),
								decoded)) {
					ok = true;
					break;
				}
			}
		}

		if (ok) {
			selected = info;
			board_id = probe_board_id;
			return h;
		}
		rayneo_send_command(h, info, RAYNEO_CMD_CLOSE_IMU);
		CloseHandle(h);
	}
	return INVALID_HANDLE_VALUE;
}

// boardId 0x3A mounts the IMU at -20 deg; other boards sit level. Mirrors the
// reference driver's deviceInfo.mountXDeg handling, which overrides the static
// profile mount.
static void apply_rayneo_board_mount(device_manager *f, int board_id)
{
	const double deg = board_id == 0x3A ? -20.0 : 0.0;
	f->mount_override_cdeg.store(static_cast<int>(deg * 100.0),
				     std::memory_order_relaxed);
	if (f->debug_log.load(std::memory_order_relaxed))
		nyan_log(NYAN_LOG_INFO,
		     "[obs-nyan-real-3dof] RayNeo board id 0x%02X -> mount %+.0f deg",
		     board_id, deg);
}

void run_rayneo_hid_session(device_manager *f, uint32_t &seen_epoch)
{
	f->mount_override_cdeg.store(INT32_MIN, std::memory_order_relaxed);
	hid_interface_info info;
	int board_id = -1;
	HANDLE h = open_rayneo_hid_stream(info, board_id);
	if (h == INVALID_HANDLE_VALUE) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			nyan_log(NYAN_LOG_WARNING,
			     "[obs-nyan-real-3dof] RayNeo HID connect failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		nyan_log(NYAN_LOG_INFO, "[obs-nyan-real-3dof] RayNeo HID connected");
	if (board_id >= 0)
		apply_rayneo_board_mount(f, board_id);

	rayneo_packet_assembler assembler;
	std::vector<uint8_t> report;
	std::vector<uint8_t> packet;
	uint32_t seq = 0;
	uint64_t last_rx_ns = nyan_now_ns();
	uint64_t last_open_retry_ns = last_rx_ns;
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::rayneo_hid)
			break;

		if (!hid_read_report(h, info.input_report_len, report, 250)) {
			const uint64_t now = nyan_now_ns();
			if (now - last_rx_ns > 1200000000ULL &&
			    now - last_open_retry_ns > 1200000000ULL) {
				last_open_retry_ns = now;
				rayneo_send_command(h, info,
						    RAYNEO_CMD_ACQUIRE_DEVICE_INFO);
				std::this_thread::sleep_for(
					std::chrono::milliseconds(80));
				rayneo_send_command(h, info, RAYNEO_CMD_OPEN_IMU);
			}
			continue;
		}
		assembler.append(report.data(), report.size());
		while (assembler.next_packet(packet)) {
			int session_board_id = -1;
			if (decode_rayneo_device_info(packet.data(), packet.size(),
						      session_board_id)) {
				// Response to the stale-retry ACQUIRE, or one
				// the probe did not catch.
				apply_rayneo_board_mount(f, session_board_id);
				continue;
			}
			decoded_sensor_report decoded;
			if (!decode_rayneo_sensor_packet(packet.data(),
							 packet.size(), decoded))
				continue;
			last_rx_ns = nyan_now_ns();
			decoded.imu.seq = seq;
			decoded.mag.seq = seq;
			seq++;
			publish_sensor_samples(f,
					       decoded.has_imu ? &decoded.imu
							       : nullptr,
					       decoded.has_mag ? &decoded.mag
							       : nullptr);
			maybe_log_sensor_rate(f, rate_log, "RayNeo HID");
		}
	}

	rayneo_send_command(h, info, RAYNEO_CMD_CLOSE_IMU);
	CloseHandle(h);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		nyan_log(NYAN_LOG_WARNING, "[obs-nyan-real-3dof] RayNeo HID disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}
