// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Marker-6DoF tracker: reads the XREAL Eye tracking camera (UVC1, 512x378
// YUY2 @25fps) through Media Foundation, detects an AprilTag 36h11 with the
// apriltag library (BSD-2-Clause, deps/apriltag) and publishes the camera
// position in the tag frame as the head position. The tag works as a ruler,
// not an anchor: the tracker re-anchors the origin at every recenter and the
// screen stays where recenter put it. Feasibility numbers and the fx
// calibration are recorded in MyGlasses2.0/analysis/18 (XREAL 1S + Eye).
#include "marker_tracker.h"

// apriltag's time_util.h needs struct timeval, which WIN32_LEAN_AND_MEAN
// strips from windows.h; winsock2.h provides it.
#include <winsock2.h>

#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <windows.h>

#include <obs-module.h>
#include <util/platform.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <vector>

#include <apriltag.h>
#include <apriltag_pose.h>
#include <tag36h11.h>
#include <common/matd.h>

#include "device_manager.h"
#include "math_util.h"
#include "nyan_types.h"

// XREAL Eye UVC1 (tracking camera) native mode; doubles as the device
// selector, since the friendly name is just "UVC Camera N".
constexpr UINT32 EYE_CAM_W = 512;
constexpr UINT32 EYE_CAM_H = 378;
// Focal length calibrated on XREAL 1S + Eye hardware (50 cm tape measure,
// ~99 deg HFOV).
constexpr double EYE_CAM_FX = 219.0;

// Finds and opens the first capture device with a native 512x378 YUY2 mode.
// Returns null when the Eye UVC is absent or disabled.
static IMFSourceReader *open_eye_camera()
{
	IMFAttributes *attrs = nullptr;
	if (FAILED(MFCreateAttributes(&attrs, 1)))
		return nullptr;
	attrs->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
		       MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
	IMFActivate **devices = nullptr;
	UINT32 count = 0;
	const HRESULT hr_enum = MFEnumDeviceSources(attrs, &devices, &count);
	attrs->Release();
	if (FAILED(hr_enum))
		return nullptr;

	IMFSourceReader *result = nullptr;
	for (UINT32 i = 0; i < count && !result; ++i) {
		IMFMediaSource *source = nullptr;
		if (FAILED(devices[i]->ActivateObject(IID_PPV_ARGS(&source))))
			continue;
		IMFSourceReader *reader = nullptr;
		const HRESULT hr =
			MFCreateSourceReaderFromMediaSource(source, nullptr,
							    &reader);
		source->Release();
		if (FAILED(hr))
			continue;
		for (DWORD t = 0;; ++t) {
			IMFMediaType *type = nullptr;
			if (FAILED(reader->GetNativeMediaType(
				    static_cast<DWORD>(
					    MF_SOURCE_READER_FIRST_VIDEO_STREAM),
				    t, &type)))
				break;
			GUID subtype = {};
			UINT32 w = 0, h = 0;
			type->GetGUID(MF_MT_SUBTYPE, &subtype);
			MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &w, &h);
			if (subtype == MFVideoFormat_YUY2 && w == EYE_CAM_W &&
			    h == EYE_CAM_H &&
			    SUCCEEDED(reader->SetCurrentMediaType(
				    static_cast<DWORD>(
					    MF_SOURCE_READER_FIRST_VIDEO_STREAM),
				    nullptr, type)))
				result = reader;
			type->Release();
			if (result)
				break;
		}
		if (!result)
			reader->Release();
	}
	for (UINT32 i = 0; i < count; ++i)
		devices[i]->Release();
	CoTaskMemFree(devices);
	return result;
}

// Reads one frame and extracts the Y plane. Returns false on device errors
// (unplug, UVC disable); an empty gray buffer means "no frame yet, retry".
static bool read_gray_frame(IMFSourceReader *reader, std::vector<uint8_t> &gray)
{
	gray.clear();
	DWORD stream = 0, flags = 0;
	LONGLONG ts = 0;
	IMFSample *sample = nullptr;
	if (FAILED(reader->ReadSample(
		    static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), 0,
		    &stream, &flags, &ts, &sample)))
		return false;
	if (flags & (MF_SOURCE_READERF_ENDOFSTREAM |
		     MF_SOURCE_READERF_ERROR)) {
		if (sample)
			sample->Release();
		return false;
	}
	if (!sample)
		return true; // stream tick / gap
	IMFMediaBuffer *buffer = nullptr;
	const HRESULT hr = sample->ConvertToContiguousBuffer(&buffer);
	sample->Release();
	if (FAILED(hr))
		return false;
	BYTE *data = nullptr;
	DWORD len = 0;
	if (FAILED(buffer->Lock(&data, nullptr, &len))) {
		buffer->Release();
		return false;
	}
	const size_t pixels = static_cast<size_t>(EYE_CAM_W) * EYE_CAM_H;
	if (len >= pixels * 2) {
		gray.resize(pixels);
		for (size_t i = 0; i < pixels; ++i)
			gray[i] = data[i * 2]; // YUY2: Y0 U Y1 V
	}
	buffer->Unlock();
	buffer->Release();
	return true;
}

void marker_worker_fn(device_manager *f)
{
	const HRESULT co = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
	if (FAILED(MFStartup(MF_VERSION, MFSTARTUP_LITE))) {
		blog(LOG_WARNING,
		     "[obs-nyan-real-3dof] marker tracker: MFStartup failed");
		if (SUCCEEDED(co))
			CoUninitialize();
		return;
	}

	apriltag_family_t *family = tag36h11_create();
	apriltag_detector_t *detector = apriltag_detector_create();
	apriltag_detector_add_family(detector, family);
	detector->quad_decimate = 1.0f; // 512x378 is small enough already
	detector->nthreads = 2;

	std::vector<uint8_t> gray;
	while (!f->stop.load(std::memory_order_relaxed)) {
		IMFSourceReader *reader = open_eye_camera();
		if (!reader) {
			std::this_thread::sleep_for(
				std::chrono::milliseconds(2000));
			continue;
		}
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] marker tracker: Eye camera opened (%ux%u)",
		     EYE_CAM_W, EYE_CAM_H);

		int read_failures = 0;
		uint64_t last_log_ns = os_gettime_ns();
		unsigned frames = 0, hits = 0;
		vec3d last_pos = {};
		double last_z = 0.0;

		while (!f->stop.load(std::memory_order_relaxed)) {
			if (!read_gray_frame(reader, gray)) {
				// Brief USB stalls recover on their own (seen
				// on hardware); a steady failure stream means
				// the camera is gone.
				if (++read_failures > 50)
					break;
				std::this_thread::sleep_for(
					std::chrono::milliseconds(20));
				continue;
			}
			read_failures = 0;
			if (gray.empty())
				continue;
			frames++;

			image_u8_t img = {static_cast<int32_t>(EYE_CAM_W),
					  static_cast<int32_t>(EYE_CAM_H),
					  static_cast<int32_t>(EYE_CAM_W),
					  gray.data()};
			zarray_t *detections =
				apriltag_detector_detect(detector, &img);
			apriltag_detection_t *best = nullptr;
			for (int i = 0; i < zarray_size(detections); ++i) {
				apriltag_detection_t *det = nullptr;
				zarray_get(detections, i, &det);
				// Only the product tag id; the margin gate
				// drops weak decodes (desk patterns etc.).
				if (det->id != 0 ||
				    det->decision_margin < 30.0f)
					continue;
				if (!best ||
				    det->decision_margin > best->decision_margin)
					best = det;
			}
			if (best) {
				hits++;
				apriltag_detection_info_t info = {};
				info.det = best;
				info.tagsize =
					clampd(f->tag_size_mm.load(
						       std::memory_order_relaxed),
					       20.0, 500.0) /
					1000.0;
				info.fx = EYE_CAM_FX;
				info.fy = EYE_CAM_FX;
				info.cx = EYE_CAM_W / 2.0;
				info.cy = EYE_CAM_H / 2.0;
				apriltag_pose_t pose = {};
				estimate_tag_pose(&info, &pose);
				if (pose.R && pose.t) {
					// Camera position in the tag frame:
					// -R^T t.
					double px = 0, py = 0, pz = 0;
					for (int r = 0; r < 3; ++r) {
						const double tr =
							matd_get(pose.t, r, 0);
						px -= matd_get(pose.R, r, 0) * tr;
						py -= matd_get(pose.R, r, 1) * tr;
						pz -= matd_get(pose.R, r, 2) * tr;
					}
					// Tag frame is image-aligned: x right,
					// y down, z into the tag. An upright
					// tag facing the viewer maps to the
					// world frame (X right, Y up, Z back)
					// as below.
					const vec3d p_world = {px, -py, -pz};
					publish_marker_position(f, p_world,
								now_us32());
					last_pos = p_world;
					last_z = matd_get(pose.t, 2, 0);
				}
				if (pose.R)
					matd_destroy(pose.R);
				if (pose.t)
					matd_destroy(pose.t);
			}
			apriltag_detections_destroy(detections);

			const uint64_t now_ns = os_gettime_ns();
			if (f->debug_log.load(std::memory_order_relaxed) &&
			    now_ns - last_log_ns >= 2000000000ULL) {
				last_log_ns = now_ns;
				vec3d head = {};
				bool head_valid = false;
				{
					std::lock_guard<std::mutex> lk(
						f->state_mutex);
					head = f->pose.pos;
					head_valid = f->pose.pos_valid;
				}
				blog(LOG_INFO,
				     "[obs-nyan-real-3dof] marker 6DoF rate %u/%u z=%.3f raw=(%+.3f,%+.3f,%+.3f) head=(%+.3f,%+.3f,%+.3f)m%s",
				     hits, frames, last_z, last_pos.x,
				     last_pos.y, last_pos.z, head.x, head.y,
				     head.z, head_valid ? "" : " (no anchor)");
				frames = 0;
				hits = 0;
			}
		}
		reader->Release();
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] marker tracker: Eye camera closed");
	}

	apriltag_detector_destroy(detector);
	tag36h11_destroy(family);
	MFShutdown();
	if (SUCCEEDED(co))
		CoUninitialize();
}
