// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// VITURE One / Pro families: on-device fused euler angles over a vendor HID
// interface, started by an MCU command.
#include <obs-module.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <vector>

#include "../device_manager.h"
#include "../device_registry.h"
#include "../hid_io.h"
#include "../math_util.h"
#include "transports.h"

constexpr uint16_t VITURE_VID = 0x35CA;
// VITURE 64-byte packets: FF FC = IMU data, FF FE = MCU command; command
// 0x15 with payload 1/0 starts/stops the fused-euler stream.
constexpr uint8_t VITURE_HDR_IMU = 0xFC;
constexpr uint8_t VITURE_HDR_CMD = 0xFE;
constexpr uint16_t VITURE_CMD_IMU = 0x15;

// --- VITURE HID --------------------------------------------------------------
// VITURE glasses expose two vendor HID interfaces (usage page 0xFF00): MI_00
// streams IMU packets, MI_01 accepts MCU commands. All packets are 64 bytes:
// header FF FC = IMU data, FF FE = command; CRC-16-CCITT (poly 0x1021, init
// 0xFFFF) at offset 2 computed over everything from offset 4; payload length
// at offset 4 (LE); command id at 0x0E (LE); data at 0x12. Command 0x15 with
// one data byte starts (1) / stops (0) the fused-euler stream. Protocol facts
// from bfvogel/viture-webxr-extension and mgschwan/viture_virtual_display;
// the two sources disagree on the CRC byte order, so commands are sent in
// both orders (the device ignores packets with a bad CRC and the command is
// idempotent).

static uint16_t viture_crc16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < len; ++i) {
		crc = static_cast<uint16_t>(crc ^
					    (static_cast<uint16_t>(data[i]) << 8));
		for (int b = 0; b < 8; ++b)
			crc = (crc & 0x8000)
				      ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
				      : static_cast<uint16_t>(crc << 1);
	}
	return crc;
}

static std::vector<uint8_t> build_viture_command(uint16_t cmd_id,
						 const uint8_t *data,
						 size_t data_len, bool crc_be)
{
	std::vector<uint8_t> pkt(64, 0);
	pkt[0] = 0xFF;
	pkt[1] = VITURE_HDR_CMD;
	const uint16_t payload_len = static_cast<uint16_t>(0x0C + data_len);
	pkt[4] = static_cast<uint8_t>(payload_len & 0xFF);
	pkt[5] = static_cast<uint8_t>(payload_len >> 8);
	pkt[0x0E] = static_cast<uint8_t>(cmd_id & 0xFF);
	pkt[0x0F] = static_cast<uint8_t>(cmd_id >> 8);
	if (data_len)
		std::memcpy(&pkt[0x12], data, data_len);
	const uint16_t crc = viture_crc16(pkt.data() + 4,
					  static_cast<size_t>(payload_len) + 2);
	pkt[2] = static_cast<uint8_t>(crc_be ? (crc >> 8) : (crc & 0xFF));
	pkt[3] = static_cast<uint8_t>(crc_be ? (crc & 0xFF) : (crc >> 8));
	return pkt;
}

struct viture_iface {
	hid_interface_info info;
	HANDLE h = INVALID_HANDLE_VALUE;
};

// The MCU interface accepts the command and the IMU interface ignores it, so
// rather than depending on interface numbers the command goes to every
// interface, in both CRC byte orders.
static void viture_send_imu_enable(std::vector<viture_iface> &ifaces,
				   bool enable)
{
	const uint8_t data = enable ? 1 : 0;
	for (int crc_be = 1; crc_be >= 0; --crc_be) {
		const auto pkt = build_viture_command(VITURE_CMD_IMU, &data, 1,
						      crc_be != 0);
		for (auto &it : ifaces)
			hid_write_report(it.h, it.info.output_report_len, pkt,
					 250);
	}
}

// Strip the Windows report-id byte if present and decode a FF FC IMU packet
// into a body->world quaternion. The payload is three big-endian floats in
// degrees ordered roll, pitch, yaw (official SDK ordering); the composition
// and signs follow the reference WebXR extension's empirically confirmed
// mapping.
static bool decode_viture_packet(const uint8_t *data, size_t len, quatd &out)
{
	if (len >= 2 && data[0] == 0x00 && data[1] == 0xFF) {
		data++;
		len--;
	}
	if (len < 30 || data[0] != 0xFF || data[1] != VITURE_HDR_IMU)
		return false;
	const float roll = read_f32_be(data + 18);
	const float pitch = read_f32_be(data + 22);
	const float yaw = read_f32_be(data + 26);
	if (!std::isfinite(roll) || !std::isfinite(pitch) ||
	    !std::isfinite(yaw) || std::fabs(roll) > 720.0f ||
	    std::fabs(pitch) > 720.0f || std::fabs(yaw) > 720.0f)
		return false;
	const double d2r = PI / 180.0;
	out = quat_normalize(quat_multiply(
		quat_from_rot_z(-roll * d2r),
		quat_multiply(quat_from_yaw_y(yaw * d2r),
			      quat_from_rot_x(-pitch * d2r))));
	return true;
}

// Open every vendor interface of the detected VITURE device, request the IMU
// stream, and identify the streaming interface by probing for FF FC packets.
static bool open_viture_hid_stream(std::vector<viture_iface> &ifaces,
				   size_t &imu_index)
{
	for (const auto &info : enumerate_hid_interfaces()) {
		if (profile_for(info.model).transport !=
			    imu_transport::viture_hid ||
		    is_consumer_control_hid(info))
			continue;
		HANDLE h = open_hid_path_rw(info.path);
		if (h != INVALID_HANDLE_VALUE)
			ifaces.push_back({info, h});
	}
	if (ifaces.empty())
		return false;

	viture_send_imu_enable(ifaces, true);
	quatd q;
	std::vector<uint8_t> report;
	const uint64_t start = os_gettime_ns();
	while (os_gettime_ns() - start < 1500000000ULL) {
		for (size_t i = 0; i < ifaces.size(); ++i) {
			if (hid_read_report(ifaces[i].h,
					    ifaces[i].info.input_report_len,
					    report, 50) &&
			    decode_viture_packet(report.data(), report.size(),
						 q)) {
				imu_index = i;
				return true;
			}
		}
	}
	for (auto &it : ifaces)
		CloseHandle(it.h);
	ifaces.clear();
	return false;
}

void run_viture_hid_session(device_manager *f, uint32_t &seen_epoch,
				   uint64_t &last_detect_ns)
{
	std::vector<viture_iface> ifaces;
	size_t imu_index = 0;
	if (!open_viture_hid_stream(ifaces, imu_index)) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] VITURE HID connect failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] VITURE HID connected "
		     "(interfaces=%u, imu=%u)",
		     static_cast<unsigned>(ifaces.size()),
		     static_cast<unsigned>(imu_index));

	std::vector<uint8_t> report;
	uint64_t last_rx_ns = os_gettime_ns();
	uint64_t last_nudge_ns = last_rx_ns;
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		refresh_detected_model(f, last_detect_ns);
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::viture_hid)
			break;
		const uint64_t now_ns = os_gettime_ns();
		if (now_ns - last_rx_ns > 3000000000ULL) {
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] VITURE HID stream "
				     "stalled (no samples for 3 s); reconnecting");
			break;
		}
		// Re-request the stream if it goes quiet (e.g. after the
		// glasses resume from standby).
		if (now_ns - last_rx_ns > 1200000000ULL &&
		    now_ns - last_nudge_ns > 1200000000ULL) {
			last_nudge_ns = now_ns;
			viture_send_imu_enable(ifaces, true);
		}

		if (!hid_read_report(ifaces[imu_index].h,
				     ifaces[imu_index].info.input_report_len,
				     report, 250))
			continue;
		quatd q;
		if (!decode_viture_packet(report.data(), report.size(), q))
			continue;
		last_rx_ns = os_gettime_ns();
		publish_external_pose(f, q, now_us32());
		maybe_log_sensor_rate(f, rate_log, "VITURE HID");
	}

	viture_send_imu_enable(ifaces, false);
	for (auto &it : ifaces)
		CloseHandle(it.h);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] VITURE HID disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}
