// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// VITURE One / Pro families: on-device fused euler angles over a vendor HID
// interface, started by an MCU command.
#include "nyan_log.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <vector>

#include "nyan_time.h"

#include "device_manager.h"
#include "device_registry.h"
#include "hid_io.h"
#include "math_util.h"
#include "transports.h"

constexpr uint16_t VITURE_VID = 0x35CA;
// VITURE 64-byte packets: FF FC = IMU data, FF FD = MCU response, FF FE = MCU
// command; command 0x15 with payload 1/0 starts/stops the fused-euler stream.
constexpr uint8_t VITURE_HDR_IMU = 0xFC;
constexpr uint8_t VITURE_HDR_RSP = 0xFD;
constexpr uint8_t VITURE_HDR_CMD = 0xFE;
constexpr uint16_t VITURE_CMD_IMU = 0x15;
// Resolution / SBS control, from disassembling the official Linux SDK 1.0.7
// (set_3d / get_3d_state -> native_mcu_exec/native_mcu_rsp): GET = cmd 0x07
// (no data), SET = cmd 0x08 with one ASCII byte '1' (2D 1920x1080) or '2'
// (3D SBS 3840x1080). The response payload's first byte echoes that ASCII
// state. Only the protocol facts are taken; no SDK code is used.
constexpr uint16_t VITURE_CMD_GET_3D = 0x07;
constexpr uint16_t VITURE_CMD_SET_3D = 0x08;
constexpr uint8_t VITURE_3D_OFF = 0x31; // '1' 1920x1080
constexpr uint8_t VITURE_3D_ON = 0x32;  // '2' 3840x1080 SBS

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

// Broadcast a command with one data byte to every interface, both CRC orders
// (mirrors viture_send_imu_enable). Used for the SET-3D command.
static void viture_send_command(std::vector<viture_iface> &ifaces,
				uint16_t cmd_id, uint8_t data)
{
	for (int crc_be = 1; crc_be >= 0; --crc_be) {
		const auto pkt = build_viture_command(cmd_id, &data, 1,
						      crc_be != 0);
		for (auto &it : ifaces)
			hid_write_report(it.h, it.info.output_report_len, pkt,
					 250);
	}
}

// Strip the optional Windows report-id byte and, if this is an FF FD MCU
// response, return its command id and the state byte. The response payload is
// [status @0x12][state @0x13]; the official SDK skips the status byte and
// reads the state, confirmed on VITURE One hardware (status 0x00 = OK, state
// '1'/'2').
static bool decode_viture_response(const uint8_t *data, size_t len,
				   uint16_t &cmd_id, uint8_t &state)
{
	if (len >= 2 && data[0] == 0x00 && data[1] == 0xFF) {
		data++;
		len--;
	}
	if (len < 0x14 || data[0] != 0xFF || data[1] != VITURE_HDR_RSP)
		return false;
	cmd_id = read_u16_le(data + 0x0E);
	state = data[0x13];
	return true;
}

// Query the current resolution state: send GET-3D and read the FF FD response
// from any interface. Returns the ASCII state byte ('1'/'2') or -1 on failure.
static int viture_query_3d(std::vector<viture_iface> &ifaces)
{
	for (int crc_be = 1; crc_be >= 0; --crc_be) {
		const auto pkt = build_viture_command(VITURE_CMD_GET_3D, nullptr,
						      0, crc_be != 0);
		for (auto &it : ifaces)
			hid_write_report(it.h, it.info.output_report_len, pkt,
					 250);
	}
	std::vector<uint8_t> report;
	const uint64_t start = nyan_now_ns();
	while (nyan_now_ns() - start < 500000000ULL) {
		for (auto &it : ifaces) {
			if (!hid_read_report(it.h, it.info.input_report_len,
					     report, 50))
				continue;
			uint16_t cmd_id = 0;
			uint8_t state = 0;
			if (decode_viture_response(report.data(), report.size(),
						   cmd_id, state) &&
			    cmd_id == VITURE_CMD_GET_3D)
				return state;
		}
	}
	return -1;
}

// Refresh display_mode_current from a GET. A failed query (e.g. while the
// display re-clocks after a mode switch) leaves the existing value untouched,
// so an optimistic store after a SET is not clobbered.
static void viture_refresh_3d(device_manager *f,
			      std::vector<viture_iface> &ifaces)
{
	const int state = viture_query_3d(ifaces);
	if (state >= 0)
		f->display_mode_current.store(state, std::memory_order_relaxed);
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
	const uint64_t start = nyan_now_ns();
	while (nyan_now_ns() - start < 1500000000ULL) {
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

void run_viture_hid_session(device_manager *f, uint32_t &seen_epoch)
{
	std::vector<viture_iface> ifaces;
	size_t imu_index = 0;
	if (!open_viture_hid_stream(ifaces, imu_index)) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			nyan_log(NYAN_LOG_WARNING,
			     "[obs-nyan-real-3dof] VITURE HID connect failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		nyan_log(NYAN_LOG_INFO,
		     "[obs-nyan-real-3dof] VITURE HID connected "
		     "(interfaces=%u, imu=%u)",
		     static_cast<unsigned>(ifaces.size()),
		     static_cast<unsigned>(imu_index));

	// The MCU command channel rides the same interfaces as the IMU stream,
	// so the resolution state is available without a separate handle.
	// Default to unavailable, then let the GET set the real value.
	f->display_mode_current.store(-1, std::memory_order_relaxed);
	viture_refresh_3d(f, ifaces);

	std::vector<uint8_t> report;
	uint64_t last_rx_ns = nyan_now_ns();
	uint64_t last_nudge_ns = last_rx_ns;
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::viture_hid)
			break;
		const uint64_t now_ns = nyan_now_ns();
		if (now_ns - last_rx_ns > 3000000000ULL) {
			if (f->debug_log.load(std::memory_order_relaxed))
				nyan_log(NYAN_LOG_WARNING,
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

		// Resolution change requested from the dock (display_mode value
		// is the ASCII state byte '1'/'2'). Switching re-clocks the
		// display, so the brief query that follows is invisible.
		const int mode_req =
			f->display_mode_request.exchange(-1, std::memory_order_relaxed);
		if (mode_req == VITURE_3D_OFF || mode_req == VITURE_3D_ON) {
			viture_send_command(ifaces, VITURE_CMD_SET_3D,
					    static_cast<uint8_t>(mode_req));
			// Store the requested state right away: the switch
			// re-clocks the display and the device can be briefly
			// unresponsive to the verify GET that follows.
			f->display_mode_current.store(mode_req,
						      std::memory_order_relaxed);
			if (f->debug_log.load(std::memory_order_relaxed))
				nyan_log(NYAN_LOG_INFO,
				     "[obs-nyan-real-3dof] VITURE 3D -> '%c'",
				     mode_req);
			viture_refresh_3d(f, ifaces);
			last_rx_ns = nyan_now_ns(); // query consumed read time
		}

		if (!hid_read_report(ifaces[imu_index].h,
				     ifaces[imu_index].info.input_report_len,
				     report, 250))
			continue;
		quatd q;
		if (!decode_viture_packet(report.data(), report.size(), q))
			continue;
		last_rx_ns = nyan_now_ns();
		publish_external_pose(f, q, now_us32());
		maybe_log_sensor_rate(f, rate_log, "VITURE HID");
	}

	viture_send_imu_enable(ifaces, false);
	for (auto &it : ifaces)
		CloseHandle(it.h);
	f->display_mode_current.store(-1, std::memory_order_relaxed);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		nyan_log(NYAN_LOG_WARNING, "[obs-nyan-real-3dof] VITURE HID disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}
