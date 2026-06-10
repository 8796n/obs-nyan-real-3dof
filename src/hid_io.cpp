// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
#include "hid_io.h"

#include <setupapi.h>
#include <sensorsapi.h>
#include <sensors.h>
#include <portabledevicetypes.h>

#include <obs-module.h>

#include <algorithm>
#include <cstring>
#include <cwctype>
#include <string>
#include <vector>

#include "device_registry.h"
#include "math_util.h"

static bool wcontains_ascii_ci(const std::wstring &s, const wchar_t *needle)
{
	if (!needle || !*needle)
		return true;
	const size_t n = std::wcslen(needle);
	if (s.size() < n)
		return false;
	for (size_t i = 0; i + n <= s.size(); ++i) {
		bool ok = true;
		for (size_t j = 0; j < n; ++j) {
			if (std::towupper(s[i + j]) != std::towupper(needle[j])) {
				ok = false;
				break;
			}
		}
		if (ok)
			return true;
	}
	return false;
}

bool is_consumer_control_hid(const hid_interface_info &info)
{
	return info.usage_page == 0x0C;
}

static model_id model_for_hid_interface(const hid_interface_info &info)
{
	for (size_t i = 0; i < g_device_registry.size(); ++i) {
		const device_entry &e = g_device_registry[i];
		if (info.vid != e.vid)
			continue;
		if (e.pid != 0 && info.pid != e.pid)
			continue;
		// HID IMU streaming runs over non-consumer-control HID
		// interfaces; don't let a Consumer Control collection identify
		// the device either.
		if (traits_for(e.profile.transport).hid_imu_stream &&
		    is_consumer_control_hid(info))
			continue;
		if (!e.product_contains.empty() &&
		    !wcontains_ascii_ci(info.product, e.product_contains.c_str()))
			continue;
		return static_cast<model_id>(i + 1);
	}
	return MODEL_UNKNOWN;
}

static bool read_hid_caps(HANDLE h, hid_interface_info &info)
{
	PHIDP_PREPARSED_DATA preparsed = nullptr;
	if (!HidD_GetPreparsedData(h, &preparsed))
		return false;
	HIDP_CAPS caps = {};
	const NTSTATUS status = HidP_GetCaps(preparsed, &caps);
	HidD_FreePreparsedData(preparsed);
	if (status != HIDP_STATUS_SUCCESS)
		return false;
	info.usage_page = caps.UsagePage;
	info.usage = caps.Usage;
	info.input_report_len = caps.InputReportByteLength;
	info.output_report_len = caps.OutputReportByteLength;
	return true;
}

std::vector<hid_interface_info> enumerate_hid_interfaces()
{
	std::vector<hid_interface_info> out;
	GUID hid_guid;
	HidD_GetHidGuid(&hid_guid);
	HDEVINFO dev_info = SetupDiGetClassDevsW(
		&hid_guid, nullptr, nullptr,
		DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
	if (dev_info == INVALID_HANDLE_VALUE)
		return out;

	SP_DEVICE_INTERFACE_DATA if_data = {};
	if_data.cbSize = sizeof(if_data);
	for (DWORD i = 0; SetupDiEnumDeviceInterfaces(dev_info, nullptr, &hid_guid, i, &if_data);
	     ++i) {
		DWORD needed = 0;
		SetupDiGetDeviceInterfaceDetailW(dev_info, &if_data, nullptr, 0,
						 &needed, nullptr);
		if (needed == 0)
			continue;
		std::vector<uint8_t> buf(needed);
		auto *detail =
			reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W *>(buf.data());
		detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
		if (!SetupDiGetDeviceInterfaceDetailW(dev_info, &if_data, detail,
						      needed, nullptr, nullptr))
			continue;

		hid_interface_info info;
		info.path = detail->DevicePath;
		HANDLE h = CreateFileW(detail->DevicePath, 0,
				       FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
				       OPEN_EXISTING, 0, nullptr);
		if (h == INVALID_HANDLE_VALUE)
			continue;
		HIDD_ATTRIBUTES attrs = {};
		attrs.Size = sizeof(attrs);
		if (HidD_GetAttributes(h, &attrs)) {
			info.vid = attrs.VendorID;
			info.pid = attrs.ProductID;
			wchar_t product[128] = {};
			if (HidD_GetProductString(h, product, sizeof(product)))
				info.product = product;
			read_hid_caps(h, info);
			info.model = model_for_hid_interface(info);
			out.push_back(info);
		}
		CloseHandle(h);
	}

	SetupDiDestroyDeviceInfoList(dev_info);
	return out;
}

// Enumerate present HID interfaces and return the first known model. When
// out_present is non-null, it is filled with every enumerated VID:PID plus usage
// page for debug logging.
// Sensor-API devices (e.g. MOVERIO BT-40) expose their IMU as HID sensor
// collections that the Windows Sensor class driver claims exclusively, so they
// never appear in the raw-HID enumeration above. Detect them instead by asking
// the Sensor API whether a gyrometer whose device path carries a registered
// sensor_api VID/PID is present. out_present (debug) gets the matched id.
static model_id detect_sensor_api_model(std::string *out_present = nullptr)
{
	bool any = false;
	for (const auto &e : g_device_registry) {
		if (e.profile.transport == imu_transport::sensor_api) {
			any = true;
			break;
		}
	}
	if (!any)
		return MODEL_UNKNOWN;

	const HRESULT co_hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
	const bool co_init = SUCCEEDED(co_hr);
	ISensorManager *mgr = nullptr;
	model_id found = MODEL_UNKNOWN;
	if (SUCCEEDED(CoCreateInstance(CLSID_SensorManager, nullptr,
				       CLSCTX_INPROC_SERVER,
				       IID_PPV_ARGS(&mgr))) &&
	    mgr) {
		ISensorCollection *col = nullptr;
		if (SUCCEEDED(mgr->GetSensorsByType(SENSOR_TYPE_GYROMETER_3D,
						    &col)) &&
		    col) {
			ULONG count = 0;
			col->GetCount(&count);
			for (ULONG c = 0; c < count && found == MODEL_UNKNOWN;
			     ++c) {
				ISensor *s = nullptr;
				if (FAILED(col->GetAt(c, &s)) || !s)
					continue;
				PROPVARIANT pv;
				PropVariantInit(&pv);
				std::wstring path;
				if (SUCCEEDED(s->GetProperty(
					    SENSOR_PROPERTY_DEVICE_PATH, &pv)) &&
				    pv.vt == VT_LPWSTR && pv.pwszVal) {
					path = pv.pwszVal;
					for (auto &ch : path)
						ch = static_cast<wchar_t>(
							std::towlower(ch));
				}
				PropVariantClear(&pv);
				s->Release();
				if (path.empty())
					continue;
				for (size_t i = 0; i < g_device_registry.size();
				     ++i) {
					const device_entry &e =
						g_device_registry[i];
					if (e.profile.transport !=
					    imu_transport::sensor_api)
						continue;
					wchar_t vp[32];
					swprintf(vp, 32, L"vid_%04x&pid_%04x",
						 e.vid, e.pid);
					if (path.find(vp) !=
					    std::wstring::npos) {
						found = static_cast<model_id>(
							i + 1);
						if (out_present &&
						    out_present->size() < 400) {
							char id[32];
							snprintf(id, sizeof(id),
								 "sensorapi:%04X:%04X ",
								 e.vid, e.pid);
							*out_present += id;
						}
						break;
					}
				}
			}
			col->Release();
		}
		mgr->Release();
	}
	if (co_init)
		CoUninitialize();
	return found;
}

model_id detect_hid_model(std::string *out_present)
{
	model_id found = MODEL_UNKNOWN;
	for (const auto &info : enumerate_hid_interfaces()) {
		if (out_present && out_present->size() < 400) {
			char id[32];
			snprintf(id, sizeof(id), "%04X:%04X/%04X ",
				 info.vid, info.pid, info.usage_page);
			*out_present += id;
		}
		if (found == MODEL_UNKNOWN && info.model != MODEL_UNKNOWN)
			found = info.model;
	}
	// Fall back to the Sensor API for devices that don't surface as raw HID.
	if (found == MODEL_UNKNOWN)
		found = detect_sensor_api_model(out_present);
	return found;
}

bool wait_overlapped_result(HANDLE h, OVERLAPPED &ov, DWORD timeout_ms,
				   DWORD &bytes)
{
	const DWORD wait = WaitForSingleObject(ov.hEvent, timeout_ms);
	if (wait == WAIT_OBJECT_0)
		return GetOverlappedResult(h, &ov, &bytes, FALSE) != FALSE;
	if (wait == WAIT_TIMEOUT) {
		CancelIoEx(h, &ov);
		WaitForSingleObject(ov.hEvent, 50);
	}
	return false;
}

bool hid_write_report(HANDLE h, USHORT output_len,
			     const std::vector<uint8_t> &payload, DWORD timeout_ms)
{
	const size_t report_len =
		std::max<size_t>(output_len ? output_len : 0, payload.size() + 1);
	std::vector<uint8_t> report(report_len, 0);
	std::memcpy(report.data() + 1, payload.data(), payload.size());

	OVERLAPPED ov = {};
	ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
	if (!ov.hEvent)
		return false;
	DWORD bytes = 0;
	BOOL ok = WriteFile(h, report.data(), static_cast<DWORD>(report.size()), nullptr,
			    &ov);
	if (!ok && GetLastError() == ERROR_IO_PENDING)
		ok = wait_overlapped_result(h, ov, timeout_ms, bytes);
	else if (ok)
		ok = GetOverlappedResult(h, &ov, &bytes, FALSE);
	CloseHandle(ov.hEvent);
	return ok != FALSE;
}

bool hid_read_report(HANDLE h, USHORT input_len, std::vector<uint8_t> &data,
			    DWORD timeout_ms)
{
	const size_t report_len = std::max<size_t>(input_len ? input_len : 0, 65);
	std::vector<uint8_t> report(report_len, 0);

	OVERLAPPED ov = {};
	ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
	if (!ov.hEvent)
		return false;
	DWORD bytes = 0;
	BOOL ok = ReadFile(h, report.data(), static_cast<DWORD>(report.size()), nullptr,
			   &ov);
	if (!ok && GetLastError() == ERROR_IO_PENDING)
		ok = wait_overlapped_result(h, ov, timeout_ms, bytes);
	else if (ok)
		ok = GetOverlappedResult(h, &ov, &bytes, FALSE);
	CloseHandle(ov.hEvent);
	if (!ok || bytes == 0)
		return false;

	size_t offset = 0;
	if (bytes > 2 && report[0] == 0) {
		const uint16_t sig_at_one = read_u16_le(report.data() + 1);
		if (sig_at_one == 0x0201 || sig_at_one == 0x53AA || report[1] == 0xAA)
			offset = 1;
	}
	data.assign(report.begin() + static_cast<ptrdiff_t>(offset),
		    report.begin() + static_cast<ptrdiff_t>(bytes));
	return true;
}

HANDLE open_hid_path_rw(const std::wstring &path)
{
	return CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE,
			   FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
			   FILE_FLAG_OVERLAPPED, nullptr);
}
