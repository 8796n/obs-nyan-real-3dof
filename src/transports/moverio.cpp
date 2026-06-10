// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// EPSON MOVERIO BT-40: IMU via the Windows Sensor API (the device enumerates
// standard HID sensor collections) plus display brightness / auto-brightness
// control over the USB serial command port.
#include <windows.h>
#include <setupapi.h>
#include <sensorsapi.h>
#include <sensors.h>
#include <portabledevicetypes.h>

#include <initguid.h>
// Not in a header we already pull in; value from ntddser.h.
DEFINE_GUID(GUID_DEVINTERFACE_COMPORT, 0x86e0d1e0, 0x8089, 0x11d0, 0x9c, 0xe4,
	    0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73);

#include <obs-module.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cwctype>
#include <string>
#include <thread>
#include <vector>

#include "../device_manager.h"
#include "../device_registry.h"
#include "../hid_io.h"
#include "../math_util.h"
#include "transports.h"

// --- Windows Sensor API transport (EPSON MOVERIO BT-40) ---------------------
// The BT-40 exposes its IMU as standard HID sensor collections, so the data is
// read through the Windows Sensor API instead of a custom protocol (the
// device's COM port only carries display-control commands such as setbright).
// Axes per the MOVERIO Basic Function SDK developer's guide: gyro/compass use
// X-right / Y-up / Z-toward-the-wearer, while the accelerometer alone is
// inverted (X-left / Y-down / Z-forward), so it is negated into the shared
// frame.

static bool propvariant_to_double(const PROPVARIANT &pv, double &out)
{
	switch (pv.vt) {
	case VT_R8:
		out = pv.dblVal;
		return true;
	case VT_R4:
		out = pv.fltVal;
		return true;
	case VT_I4:
		out = pv.lVal;
		return true;
	case VT_UI4:
		out = pv.ulVal;
		return true;
	default:
		return false;
	}
}

// Read a 3-axis value from the sensor's latest report. Returns true only when
// the report timestamp advanced (the API hands back the same report between
// hardware updates) and all three fields decoded; last_ft100 holds the report
// FILETIME in 100 ns units for timestamping the sample.
static bool sensor_read_vec3(ISensor *sensor, const PROPERTYKEY &kx,
			     const PROPERTYKEY &ky, const PROPERTYKEY &kz,
			     uint64_t &last_ft100, vec3d &out)
{
	ISensorDataReport *report = nullptr;
	if (FAILED(sensor->GetData(&report)) || !report)
		return false;

	SYSTEMTIME st = {};
	FILETIME ft = {};
	bool fresh = false;
	if (SUCCEEDED(report->GetTimestamp(&st)) &&
	    SystemTimeToFileTime(&st, &ft)) {
		const uint64_t t =
			(static_cast<uint64_t>(ft.dwHighDateTime) << 32) |
			ft.dwLowDateTime;
		fresh = t != last_ft100;
		if (fresh)
			last_ft100 = t;
	}

	bool ok = fresh;
	if (fresh) {
		const PROPERTYKEY keys[3] = {kx, ky, kz};
		double v[3] = {};
		for (int i = 0; i < 3 && ok; ++i) {
			PROPVARIANT pv;
			PropVariantInit(&pv);
			ok = SUCCEEDED(report->GetSensorValue(keys[i], &pv)) &&
			     propvariant_to_double(pv, v[i]);
			PropVariantClear(&pv);
		}
		if (ok)
			out = {v[0], v[1], v[2]};
	}
	report->Release();
	return ok;
}

static void sensor_request_min_interval(ISensor *sensor)
{
	PROPVARIANT pv;
	PropVariantInit(&pv);
	ULONG min_ms = 0;
	if (SUCCEEDED(sensor->GetProperty(SENSOR_PROPERTY_MIN_REPORT_INTERVAL,
					  &pv)) &&
	    pv.vt == VT_UI4)
		min_ms = pv.ulVal;
	PropVariantClear(&pv);

	IPortableDeviceValues *props = nullptr;
	if (FAILED(CoCreateInstance(CLSID_PortableDeviceValues, nullptr,
				    CLSCTX_INPROC_SERVER,
				    IID_PPV_ARGS(&props))))
		return;
	props->SetUnsignedIntegerValue(SENSOR_PROPERTY_CURRENT_REPORT_INTERVAL,
				       min_ms ? min_ms : 4);
	IPortableDeviceValues *results = nullptr;
	sensor->SetProperties(props, &results);
	if (results)
		results->Release();
	props->Release();
}

// ISensor::GetData can block indefinitely while the device sleeps (MOVERIO
// mutes its display and stops sensor reports after sitting still), which
// would also wedge the watchdog. Only call GetData when the sensor reports
// SENSOR_STATE_READY; GetState is a cheap non-blocking query.
static bool sensor_is_ready(ISensor *sensor)
{
	SensorState state = SENSOR_STATE_ERROR;
	return SUCCEEDED(sensor->GetState(&state)) &&
	       state == SENSOR_STATE_READY;
}

// Pick the sensor of the given type whose device path contains the detected
// USB identity, so a sensor on another device (laptop IMU etc.) is never used.
static ISensor *find_device_sensor(ISensorManager *mgr, REFSENSOR_TYPE_ID type,
				   const std::wstring &vidpid)
{
	ISensorCollection *col = nullptr;
	if (FAILED(mgr->GetSensorsByType(type, &col)) || !col)
		return nullptr;
	ULONG count = 0;
	col->GetCount(&count);
	ISensor *found = nullptr;
	for (ULONG i = 0; i < count && !found; ++i) {
		ISensor *s = nullptr;
		if (FAILED(col->GetAt(i, &s)) || !s)
			continue;
		PROPVARIANT pv;
		PropVariantInit(&pv);
		bool match = false;
		if (SUCCEEDED(s->GetProperty(SENSOR_PROPERTY_DEVICE_PATH, &pv)) &&
		    pv.vt == VT_LPWSTR && pv.pwszVal) {
			std::wstring path = pv.pwszVal;
			for (auto &c : path)
				c = static_cast<wchar_t>(std::towlower(c));
			match = path.find(vidpid) != std::wstring::npos;
		}
		PropVariantClear(&pv);
		if (match)
			found = s;
		else
			s->Release();
	}
	col->Release();
	return found;
}

// --- MOVERIO display-control serial port ------------------------------------
// The BT-40 family has no hardware brightness control; it accepts the text
// commands documented in the MOVERIO Basic Function SDK's Windows command
// reference over a USB-CDC COM port (setbright 0-20 / 50 = auto, getbright).

static bool find_device_com_port(uint16_t vid, uint16_t pid, std::wstring &out)
{
	HDEVINFO devs = SetupDiGetClassDevsW(&GUID_DEVINTERFACE_COMPORT, nullptr,
					     nullptr,
					     DIGCF_PRESENT |
						     DIGCF_DEVICEINTERFACE);
	if (devs == INVALID_HANDLE_VALUE)
		return false;
	wchar_t want[32];
	swprintf(want, 32, L"vid_%04x&pid_%04x", vid, pid);
	bool found = false;
	SP_DEVINFO_DATA dev = {};
	dev.cbSize = sizeof(dev);
	for (DWORD i = 0; !found && SetupDiEnumDeviceInfo(devs, i, &dev); ++i) {
		wchar_t inst[512] = L"";
		if (!SetupDiGetDeviceInstanceIdW(devs, &dev, inst, 512, nullptr))
			continue;
		for (wchar_t *p = inst; *p; ++p)
			*p = static_cast<wchar_t>(std::towlower(*p));
		if (!wcsstr(inst, want))
			continue;
		HKEY key = SetupDiOpenDevRegKey(devs, &dev, DICS_FLAG_GLOBAL, 0,
						DIREG_DEV, KEY_READ);
		if (key == INVALID_HANDLE_VALUE)
			continue;
		wchar_t port[64] = L"";
		DWORD len = sizeof(port);
		DWORD type = 0;
		if (RegQueryValueExW(key, L"PortName", nullptr, &type,
				     reinterpret_cast<LPBYTE>(port), &len) ==
			    ERROR_SUCCESS &&
		    type == REG_SZ && port[0]) {
			out = L"\\\\.\\" + std::wstring(port);
			found = true;
		}
		RegCloseKey(key);
	}
	SetupDiDestroyDeviceInfoList(devs);
	return found;
}

static HANDLE open_moverio_serial(uint16_t vid, uint16_t pid)
{
	std::wstring path;
	if (!find_device_com_port(vid, pid, path))
		return INVALID_HANDLE_VALUE;
	HANDLE h = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
			       nullptr, OPEN_EXISTING, 0, nullptr);
	if (h == INVALID_HANDLE_VALUE)
		return h;
	DCB dcb = {};
	dcb.DCBlength = sizeof(dcb);
	if (GetCommState(h, &dcb)) {
		dcb.BaudRate = CBR_115200; // nominal; USB-CDC ignores the rate
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;
		SetCommState(h, &dcb);
	}
	COMMTIMEOUTS to = {};
	to.ReadIntervalTimeout = 50;
	to.ReadTotalTimeoutConstant = 250;
	to.WriteTotalTimeoutConstant = 250;
	SetCommTimeouts(h, &to);
	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
	return h;
}

static bool moverio_serial_command(HANDLE h, const char *cmd, char *reply,
				   size_t reply_len)
{
	char buf[64];
	const int n = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
	if (n <= 0)
		return false;
	DWORD written = 0;
	if (!WriteFile(h, buf, static_cast<DWORD>(n), &written, nullptr))
		return false;
	if (!reply || reply_len == 0)
		return true;
	DWORD got = 0;
	if (!ReadFile(h, reply, static_cast<DWORD>(reply_len - 1), &got,
		      nullptr))
		return false;
	reply[got] = '\0';
	return got > 0;
}

// Query commands reply with a number as text ("0".."20", "50" = auto for
// getbright; "0"/"1" for getautobright). Framing is undocumented and the
// device may echo the command line first, so scan for the first digit and
// retry the read once when only the echo arrived in the first chunk.
static int moverio_query_value(HANDLE h, const char *cmd, int max_valid)
{
	char reply[64] = "";
	if (!moverio_serial_command(h, cmd, reply, sizeof(reply)))
		return -1;
	const char *p = reply;
	while (*p && !std::isdigit(static_cast<unsigned char>(*p)))
		++p;
	char more[32] = "";
	if (!*p) {
		DWORD got = 0;
		if (!ReadFile(h, more, sizeof(more) - 1, &got, nullptr) || !got)
			return -1;
		more[got] = '\0';
		p = more;
		while (*p && !std::isdigit(static_cast<unsigned char>(*p)))
			++p;
		if (!*p)
			return -1;
	}
	const int v = std::atoi(p);
	return (v >= 0 && v <= max_valid) ? v : -1;
}

static int moverio_query_brightness(HANDLE h)
{
	return moverio_query_value(h, "getbright", 50);
}

static int moverio_query_autobright(HANDLE h)
{
	return moverio_query_value(h, "getautobright", 1);
}

void run_sensor_api_session(device_manager *f, uint32_t &seen_epoch,
				   uint64_t &last_detect_ns)
{
	const model_id m = detected_hid_model(f);
	if (m == MODEL_UNKNOWN)
		return;
	const device_entry &entry =
		g_device_registry[static_cast<size_t>(m) - 1];
	wchar_t vidpid[32];
	swprintf(vidpid, 32, L"vid_%04x&pid_%04x", entry.vid, entry.pid);

	const HRESULT co_hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
	const bool co_init = SUCCEEDED(co_hr);

	ISensorManager *mgr = nullptr;
	ISensor *gyro = nullptr;
	ISensor *accel = nullptr;
	ISensor *mag = nullptr;
	if (SUCCEEDED(CoCreateInstance(CLSID_SensorManager, nullptr,
				       CLSCTX_INPROC_SERVER,
				       IID_PPV_ARGS(&mgr))) &&
	    mgr) {
		gyro = find_device_sensor(mgr, SENSOR_TYPE_GYROMETER_3D, vidpid);
		accel = find_device_sensor(mgr, SENSOR_TYPE_ACCELEROMETER_3D,
					   vidpid);
		mag = find_device_sensor(mgr, SENSOR_TYPE_COMPASS_3D, vidpid);
	}

	// A sleeping device (display mute by stillness/tap detection) leaves
	// the sensors enumerable but not READY; treat that as not-connected so
	// we retry instead of blocking in GetData.
	if (!gyro || !accel || !sensor_is_ready(gyro)) {
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] Sensor API connect failed "
			     "(gyro=%d accel=%d ready=%d)",
			     gyro != nullptr, accel != nullptr,
			     gyro ? sensor_is_ready(gyro) : 0);
		if (gyro)
			gyro->Release();
		if (accel)
			accel->Release();
		if (mag)
			mag->Release();
		if (mgr)
			mgr->Release();
		if (co_init)
			CoUninitialize();
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	sensor_request_min_interval(gyro);
	sensor_request_min_interval(accel);
	if (mag)
		sensor_request_min_interval(mag);

	// Require a live gyro sample before declaring the session connected.
	// A wedged/sleeping device leaves the sensors enumerable and READY but
	// silent; without this gate the session flaps connected/disconnected
	// every watchdog period and the dock controls flicker with it.
	const double deg_to_rad = PI / 180.0;
	uint64_t gyro_ft100 = 0;
	vec3d g_latest = {};
	bool live = false;
	const uint64_t probe_start = os_gettime_ns();
	while (!f->stop.load(std::memory_order_relaxed) &&
	       os_gettime_ns() - probe_start < 1500000000ULL) {
		if (!sensor_is_ready(gyro)) {
			std::this_thread::sleep_for(
				std::chrono::milliseconds(50));
			continue;
		}
		if (sensor_read_vec3(
			    gyro,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_X_DEGREES_PER_SECOND,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Y_DEGREES_PER_SECOND,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Z_DEGREES_PER_SECOND,
			    gyro_ft100, g_latest)) {
			live = true;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	if (!live) {
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] Sensor API found sensors but "
			     "no live samples (device asleep or sensor stack "
			     "wedged); retrying");
		gyro->Release();
		accel->Release();
		if (mag)
			mag->Release();
		mgr->Release();
		if (co_init)
			CoUninitialize();
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] Sensor API connected (mag=%d)",
		     mag != nullptr);

	// Display-control serial port for brightness. Optional: the IMU works
	// without it and the dock disables the brightness row when absent.
	HANDLE serial = open_moverio_serial(entry.vid, entry.pid);
	f->brightness_current.store(
		serial != INVALID_HANDLE_VALUE ? moverio_query_brightness(serial)
					       : -1,
		std::memory_order_relaxed);
	f->autobright_current.store(
		serial != INVALID_HANDLE_VALUE ? moverio_query_autobright(serial)
					       : -1,
		std::memory_order_relaxed);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] MOVERIO serial %s (brightness %d, "
		     "autobright %d)",
		     serial != INVALID_HANDLE_VALUE ? "opened" : "not available",
		     f->brightness_current.load(std::memory_order_relaxed),
		     f->autobright_current.load(std::memory_order_relaxed));
	if (serial != INVALID_HANDLE_VALUE &&
	    f->debug_log.load(std::memory_order_relaxed) &&
	    f->brightness_current.load(std::memory_order_relaxed) < 0) {
		// Reply framing is undocumented; dump one raw reply to make
		// parser failures diagnosable from the log.
		char raw[64] = "";
		moverio_serial_command(serial, "getbright", raw, sizeof(raw));
		for (char *p = raw; *p; ++p)
			if (!std::isprint(static_cast<unsigned char>(*p)))
				*p = '.';
		blog(LOG_INFO, "[obs-nyan-real-3dof] getbright raw reply: '%s'",
		     raw);
	}

	uint64_t accel_ft100 = 0;
	uint64_t mag_ft100 = 0;
	vec3d a_latest = {};
	bool have_accel = false;
	uint32_t seq = 0;
	uint64_t last_rx_ns = os_gettime_ns();
	rate_log_state rate_log;

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		refresh_detected_model(f, last_detect_ns);
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::sensor_api)
			break;
		const int bright_req = f->brightness_request.exchange(
			-1, std::memory_order_relaxed);
		if (bright_req >= 0 && serial != INVALID_HANDLE_VALUE) {
			char cmd[32];
			char reply[32] = "";
			snprintf(cmd, sizeof(cmd), "setbright %d", bright_req);
			moverio_serial_command(serial, cmd, reply, sizeof(reply));
			// Re-read instead of trusting the reply framing.
			const int now_bright = moverio_query_brightness(serial);
			f->brightness_current.store(
				now_bright >= 0 ? now_bright : bright_req,
				std::memory_order_relaxed);
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_INFO,
				     "[obs-nyan-real-3dof] setbright %d -> "
				     "reply '%s', getbright %d",
				     bright_req, reply, now_bright);
		}

		const int auto_req = f->autobright_request.exchange(
			-1, std::memory_order_relaxed);
		if (auto_req >= 0 && serial != INVALID_HANDLE_VALUE) {
			char cmd[32];
			char reply[32] = "";
			snprintf(cmd, sizeof(cmd), "enableautobright %d",
				 auto_req ? 1 : 0);
			moverio_serial_command(serial, cmd, reply, sizeof(reply));
			const int now_auto = moverio_query_autobright(serial);
			f->autobright_current.store(
				now_auto >= 0 ? now_auto : auto_req,
				std::memory_order_relaxed);
			// Leaving auto mode restores a manual level (and auto
			// mode reports 50); refresh the slider either way.
			const int now_bright = moverio_query_brightness(serial);
			if (now_bright >= 0)
				f->brightness_current.store(
					now_bright, std::memory_order_relaxed);
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_INFO,
				     "[obs-nyan-real-3dof] enableautobright %d "
				     "-> reply '%s', getautobright %d, "
				     "getbright %d",
				     auto_req, reply, now_auto, now_bright);
		}

		// The sensor objects go quiet rather than erroring when the
		// device misbehaves; reconnect like the other transports.
		if (os_gettime_ns() - last_rx_ns > 3000000000ULL) {
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] Sensor API stream "
				     "stalled (no samples for 3 s); reconnecting");
			break;
		}

		// While the device sleeps GetData would block indefinitely;
		// idle here and let the stall watchdog cycle the session.
		if (!sensor_is_ready(gyro)) {
			std::this_thread::sleep_for(
				std::chrono::milliseconds(50));
			continue;
		}

		if (sensor_read_vec3(accel, SENSOR_DATA_TYPE_ACCELERATION_X_G,
				     SENSOR_DATA_TYPE_ACCELERATION_Y_G,
				     SENSOR_DATA_TYPE_ACCELERATION_Z_G,
				     accel_ft100, a_latest))
			have_accel = true;

		if (sensor_read_vec3(
			    gyro,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_X_DEGREES_PER_SECOND,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Y_DEGREES_PER_SECOND,
			    SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Z_DEGREES_PER_SECOND,
			    gyro_ft100, g_latest) &&
		    have_accel) {
			last_rx_ns = os_gettime_ns();
			imu_sample imu;
			imu.gx = static_cast<float>(g_latest.x * deg_to_rad);
			imu.gy = static_cast<float>(g_latest.y * deg_to_rad);
			imu.gz = static_cast<float>(g_latest.z * deg_to_rad);
			// The accelerometer frame is the inverse of the
			// gyro/compass frame on MOVERIO; negate into one frame.
			imu.ax = static_cast<float>(-a_latest.x);
			imu.ay = static_cast<float>(-a_latest.y);
			imu.az = static_cast<float>(-a_latest.z);
			imu.ts_us = static_cast<uint32_t>(gyro_ft100 / 10);
			imu.seq = seq++;
			publish_sensor_samples(f, &imu, nullptr);
			maybe_log_sensor_rate(f, rate_log, "Sensor API");
		}

		vec3d mg = {};
		if (mag &&
		    sensor_read_vec3(
			    mag,
			    SENSOR_DATA_TYPE_MAGNETIC_FIELD_STRENGTH_X_MILLIGAUSS,
			    SENSOR_DATA_TYPE_MAGNETIC_FIELD_STRENGTH_Y_MILLIGAUSS,
			    SENSOR_DATA_TYPE_MAGNETIC_FIELD_STRENGTH_Z_MILLIGAUSS,
			    mag_ft100, mg)) {
			mag_sample ms;
			ms.mx = static_cast<float>(mg.x);
			ms.my = static_cast<float>(mg.y);
			ms.mz = static_cast<float>(mg.z);
			ms.temp_c = 0.0f;
			ms.ts_us = static_cast<uint32_t>(mag_ft100 / 10);
			ms.seq = seq;
			publish_sensor_samples(f, nullptr, &ms);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}

	if (serial != INVALID_HANDLE_VALUE)
		CloseHandle(serial);
	f->brightness_current.store(-1, std::memory_order_relaxed);
	f->autobright_current.store(-1, std::memory_order_relaxed);
	gyro->Release();
	accel->Release();
	if (mag)
		mag->Release();
	mgr->Release();
	if (co_init)
		CoUninitialize();
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] Sensor API disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}
