// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Nreal Light: raw gyro/accel packets from the OV580 camera/IMU coprocessor's
// vendor HID interface, started by a short command.
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

// The Nreal Light is a collection of separate USB devices; the 6DoF IMU lives
// on the OmniVision OV580 camera coprocessor (05A9:0680), not on the STM32
// MCU that handles brightness/mode commands. Protocol facts from the
// ar-drivers-rs project's nreal_light.rs, confirmed on Nreal Light hardware
// (MyGlasses2.0/analysis/13, 2026-06): commands are 7-byte reports
// [report id 0x02, cmd, subcmd, 0...]; cmd 0x19 subcmd 1/0 starts/stops the
// IMU stream. The stream then delivers 1000 Hz input reports with id 0x01:
// at offset 44 (counting the report id) u64 LE gyro timestamp in ns, u32 LE
// multiplier and divisor, i32 LE x/y/z in units of mul/div deg/s, then the
// same u64+u32+u32+i32x3 block for the accelerometer in units of mul/div g.
constexpr uint16_t NREAL_OV580_VID = 0x05A9;
constexpr uint8_t NREAL_REPORT_IMU = 0x01;
constexpr uint8_t NREAL_REPORT_CMD = 0x02;
constexpr uint8_t NREAL_CMD_IMU_STREAM = 0x19;
constexpr size_t NREAL_IMU_PAYLOAD_OFF = 44;
constexpr size_t NREAL_IMU_PAYLOAD_LEN = 56; // two ts+mul+div+xyz blocks

// The OV580 uses numbered HID reports (the command report id is 0x02), so the
// shared hid_write_report helper, which prepends the report-id byte 0 for
// unnumbered devices, does not fit; this variant sends the payload as-is,
// zero-padded to the interface's output report length.
static bool ov580_write_report(HANDLE h, USHORT output_len,
			       const std::vector<uint8_t> &report_with_id,
			       DWORD timeout_ms)
{
	const size_t report_len = std::max<size_t>(
		output_len ? output_len : 0, report_with_id.size());
	std::vector<uint8_t> report(report_len, 0);
	std::memcpy(report.data(), report_with_id.data(), report_with_id.size());

	OVERLAPPED ov = {};
	ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
	if (!ov.hEvent)
		return false;
	DWORD bytes = 0;
	BOOL ok = WriteFile(h, report.data(), static_cast<DWORD>(report.size()),
			    nullptr, &ov);
	if (!ok && GetLastError() == ERROR_IO_PENDING)
		ok = wait_overlapped_result(h, ov, timeout_ms, bytes);
	else if (ok)
		ok = GetOverlappedResult(h, &ov, &bytes, FALSE);
	CloseHandle(ov.hEvent);
	return ok != FALSE;
}

static void nreal_send_imu_stream(HANDLE h, USHORT output_len, bool enable)
{
	const std::vector<uint8_t> pkt = {NREAL_REPORT_CMD, NREAL_CMD_IMU_STREAM,
					  static_cast<uint8_t>(enable ? 1 : 0),
					  0, 0, 0, 0};
	ov580_write_report(h, output_len, pkt, 250);
}

// Decode a report-id 0x01 IMU packet into the shared X-right / Y-up /
// Z-toward-wearer frame in SI units: the sensor's native frame maps as
// (x, -y, -z), gyro deg/s -> rad/s, accel g -> m/s^2 (ar-drivers-rs's
// empirically confirmed mapping). The per-device biases stored in the OV580
// config blob are not read; the tracker estimates gyro bias itself during
// stationary calibration.
static bool decode_nreal_imu_packet(const uint8_t *data, size_t len,
				    imu_sample &out)
{
	if (!data || len < NREAL_IMU_PAYLOAD_OFF + NREAL_IMU_PAYLOAD_LEN ||
	    data[0] != NREAL_REPORT_IMU)
		return false;
	const uint8_t *p = data + NREAL_IMU_PAYLOAD_OFF;
	const uint64_t gyro_ts_ns = read_u64_le(p);
	const double gyro_mul = read_u32_le(p + 8);
	const double gyro_div = read_u32_le(p + 12);
	const double acc_mul = read_u32_le(p + 28 + 8);
	const double acc_div = read_u32_le(p + 28 + 12);
	if (gyro_mul <= 0.0 || gyro_div <= 0.0 || acc_mul <= 0.0 ||
	    acc_div <= 0.0)
		return false;

	const double d2r = PI / 180.0;
	const double gs = gyro_mul / gyro_div * d2r;
	const double as = acc_mul / acc_div * 9.81;
	const double gx = read_i32_le(p + 16) * gs;
	const double gy = -(read_i32_le(p + 20) * gs);
	const double gz = -(read_i32_le(p + 24) * gs);
	const double ax = read_i32_le(p + 28 + 16) * as;
	const double ay = -(read_i32_le(p + 28 + 20) * as);
	const double az = -(read_i32_le(p + 28 + 24) * as);

	// ~70 rad/s is the +-2000 dps sensor limit; 320 m/s^2 is +-32 g.
	if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz) ||
	    !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
	    std::fabs(gx) > 70.0 || std::fabs(gy) > 70.0 ||
	    std::fabs(gz) > 70.0 || std::fabs(ax) > 320.0 ||
	    std::fabs(ay) > 320.0 || std::fabs(az) > 320.0)
		return false;

	out.gx = static_cast<float>(gx);
	out.gy = static_cast<float>(gy);
	out.gz = static_cast<float>(gz);
	out.ax = static_cast<float>(ax);
	out.ay = static_cast<float>(ay);
	out.az = static_cast<float>(az);
	out.ts_us = static_cast<uint32_t>(gyro_ts_ns / 1000ULL);
	return true;
}

// Open the OV580 vendor HID interface of a detected nreal_hid model (the
// model also matches the MCU's HID interface; only the OV580 streams IMU
// data), request the stream and verify one decodable packet arrives.
static HANDLE open_nreal_hid_stream(hid_interface_info &selected)
{
	for (const auto &info : enumerate_hid_interfaces()) {
		if (profile_for(info.model).transport != imu_transport::nreal_hid ||
		    info.vid != NREAL_OV580_VID || is_consumer_control_hid(info))
			continue;
		HANDLE h = open_hid_path_rw(info.path);
		if (h == INVALID_HANDLE_VALUE)
			continue;

		nreal_send_imu_stream(h, info.output_report_len, true);
		bool ok = false;
		imu_sample sample;
		std::vector<uint8_t> report;
		const uint64_t start = os_gettime_ns();
		while (!ok && os_gettime_ns() - start < 1500000000ULL) {
			if (!hid_read_report(h, info.input_report_len, report,
					     100))
				continue;
			if (decode_nreal_imu_packet(report.data(), report.size(),
						    sample))
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

void run_nreal_hid_session(device_manager *f, uint32_t &seen_epoch)
{
	hid_interface_info info;
	HANDLE h = open_nreal_hid_stream(info);
	if (h == INVALID_HANDLE_VALUE) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] Nreal HID connect failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO, "[obs-nyan-real-3dof] Nreal HID connected");

	std::vector<uint8_t> report;
	uint32_t seq = 0;
	uint64_t last_rx_ns = os_gettime_ns();
	uint64_t last_nudge_ns = last_rx_ns;
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::nreal_hid)
			break;
		const uint64_t now_ns = os_gettime_ns();
		// The glasses power the whole USB hub down after their sleep
		// timeout, so a stall usually precedes the device vanishing;
		// reconnect like the other transports.
		if (now_ns - last_rx_ns > 3000000000ULL) {
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] Nreal HID stream "
				     "stalled (no samples for 3 s); reconnecting");
			break;
		}
		// Re-request the stream if it goes quiet (e.g. after the
		// OV580 is reset by the MCU).
		if (now_ns - last_rx_ns > 1200000000ULL &&
		    now_ns - last_nudge_ns > 1200000000ULL) {
			last_nudge_ns = now_ns;
			nreal_send_imu_stream(h, info.output_report_len, true);
		}

		if (!hid_read_report(h, info.input_report_len, report, 250))
			continue;
		imu_sample imu;
		if (!decode_nreal_imu_packet(report.data(), report.size(), imu))
			continue;
		last_rx_ns = os_gettime_ns();
		imu.seq = seq++;
		publish_sensor_samples(f, &imu, nullptr);
		maybe_log_sensor_rate(f, rate_log, "Nreal HID");
	}

	nreal_send_imu_stream(h, info.output_report_len, false);
	CloseHandle(h);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] Nreal HID disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}
