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

#include <algorithm>
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
// Kannala-Brandt intrinsics, calibrated on XREAL 1S + Eye hardware
// (MyGlasses2.0 test/calibrate-eye-fisheye.py: rms 0.162 px, residuals
// sub-pixel out to the image corners; validated flat to +2.5 mm/100 px
// off-axis and -0.8% absolute against a 0.50 m tape measure). The earlier
// tape-measure-only fx of 219 underestimated scale by ~10%.
constexpr double EYE_CAM_FX = 238.4941;
constexpr double EYE_CAM_FY = 238.7309;
constexpr double EYE_CAM_CX = 252.3826;
constexpr double EYE_CAM_CY = 189.6232;
constexpr double EYE_CAM_K[4] = {0.27099476, 0.21130490, 0.0, 0.0};

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

// Kannala-Brandt fisheye -> pinhole. The distorted radius in normalized
// coordinates is r_d = theta*(1 + k1 th^2 + k2 th^4 + k3 th^6 + k4 th^8);
// solve for theta (Newton, seeded at r_d - converges in a few steps over
// the whole field) and remap the point onto the r_u = tan(theta) pinhole
// projection the pose solver assumes.
static void undistort_point(double *u, double *v)
{
	const double xd = (*u - EYE_CAM_CX) / EYE_CAM_FX;
	const double yd = (*v - EYE_CAM_CY) / EYE_CAM_FY;
	const double rd = std::sqrt(xd * xd + yd * yd);
	if (rd < 1e-9)
		return;
	double theta = rd;
	for (int i = 0; i < 8; ++i) {
		const double t2 = theta * theta;
		const double poly =
			1.0 + t2 * (EYE_CAM_K[0] +
				    t2 * (EYE_CAM_K[1] +
					  t2 * (EYE_CAM_K[2] +
						t2 * EYE_CAM_K[3])));
		const double dpoly =
			1.0 + t2 * (3.0 * EYE_CAM_K[0] +
				    t2 * (5.0 * EYE_CAM_K[1] +
					  t2 * (7.0 * EYE_CAM_K[2] +
						t2 * 9.0 * EYE_CAM_K[3])));
		theta -= (theta * poly - rd) / dpoly;
	}
	if (!(theta > 0.0) || theta >= 1.45)
		return; // beyond ~83 deg off-axis: tan() unusable, leave it
	const double scale = std::tan(theta) / rd;
	*u = EYE_CAM_CX + EYE_CAM_FX * xd * scale;
	*v = EYE_CAM_CY + EYE_CAM_FY * yd * scale;
}

// Camera position in the tag frame (-R^T t), mapped to the world axis
// convention. Tag frame is image-aligned: x right, y down, z into the tag;
// an upright tag facing the viewer maps to X right, Y up, Z back.
static vec3d tag_world_pos(const apriltag_pose_t &pose)
{
	double px = 0, py = 0, pz = 0;
	for (int r = 0; r < 3; ++r) {
		const double tr = matd_get(pose.t, r, 0);
		px -= matd_get(pose.R, r, 0) * tr;
		py -= matd_get(pose.R, r, 1) * tr;
		pz -= matd_get(pose.R, r, 2) * tr;
	}
	return {px, -py, -pz};
}

static double dist2(const vec3d &a, const vec3d &b)
{
	const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
	return dx * dx + dy * dy + dz * dz;
}

// Euclidean camera-to-tag range. NOT the camera-frame z: the axial depth
// falls with cos(off-axis angle) when the head rotates, which once led a
// whole debugging session astray ("z swims when turning" was geometry, not
// error - see MyGlasses2.0/analysis/18).
static double tag_range(const apriltag_pose_t &pose)
{
	double s = 0.0;
	for (int r = 0; r < 3; ++r) {
		const double t = matd_get(pose.t, r, 0);
		s += t * t;
	}
	return std::sqrt(s);
}

// Tag z-axis (the tag's facing normal) in the head frame: column 2 of the
// tag->camera rotation, with the camera-to-head axis mapping applied
// (camera x right, y down, z forward -> head x right, y up, z back).
static vec3d tag_normal_head(const apriltag_pose_t &pose)
{
	return {matd_get(pose.R, 0, 2), -matd_get(pose.R, 1, 2),
		-matd_get(pose.R, 2, 2)};
}

static double dot3(const vec3d &a, const vec3d &b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
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
		vec3d last_raw = {};
		bool have_last_raw = false;
		vec3d normal_ref = {};
		bool have_normal_ref = false;
		// Oscillation diagnostics for the 2 s debug summary.
		vec3d acc_sum = {}, acc_sum2 = {};
		unsigned acc_n = 0;
		unsigned flip_picks = 0;
		double max_step_m = 0.0;

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
				for (int c = 0; c < 4; ++c)
					undistort_point(&best->p[c][0],
							&best->p[c][1]);
				undistort_point(&best->c[0], &best->c[1]);
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
				// Planar pose has two solutions whose errors
				// nearly tie at frontal views; the naive
				// lowest-error pick then alternates frame to
				// frame (a few-cm sawtooth that reads as
				// screen judder). When the errors are close,
				// prefer the solution nearer to the previous
				// frame instead.
				double err1 = 0.0, err2 = 0.0;
				apriltag_pose_t pose1 = {}, pose2 = {};
				estimate_tag_pose_orthogonal_iteration(
					&info, &err1, &pose1, &err2, &pose2,
					50);
				vec3d cand[2] = {};
				vec3d cand_n[2] = {};
				double cand_z[2] = {0.0, 0.0};
				double cand_err[2] = {0.0, 0.0};
				int n_cand = 0;
				if (pose1.R && pose1.t) {
					cand[n_cand] = tag_world_pos(pose1);
					cand_n[n_cand] = tag_normal_head(pose1);
					cand_z[n_cand] = tag_range(pose1);
					cand_err[n_cand] = err1;
					n_cand++;
				}
				if (pose2.R && pose2.t) {
					cand[n_cand] = tag_world_pos(pose2);
					cand_n[n_cand] = tag_normal_head(pose2);
					cand_z[n_cand] = tag_range(pose2);
					cand_err[n_cand] = err2;
					n_cand++;
				}
				// Rotate the candidate tag normals into the
				// recentered world: the tag is a static
				// object, so its world normal must not move.
				// The IMU orientation is precise, which makes
				// this the reliable branch discriminator the
				// reprojection errors cannot provide near
				// frontal views.
				const quatd qrel = device_pose_orientation(f);
				vec3d n_world[2] = {};
				for (int c = 0; c < n_cand; ++c)
					n_world[c] = rotate_vector(qrel,
								   cand_n[c]);
				int pick = 0;
				if (n_cand == 2) {
					const int by_err =
						cand_err[0] <= cand_err[1] ? 0
									   : 1;
					const double lo = cand_err[by_err];
					const double hi = cand_err[1 - by_err];
					const double d0 = have_normal_ref
								  ? dot3(n_world[0],
									 normal_ref)
								  : 0.0;
					const double d1 = have_normal_ref
								  ? dot3(n_world[1],
									 normal_ref)
								  : 0.0;
					if (hi > lo * 3.0)
						pick = by_err;
					else if (have_normal_ref &&
						 std::fabs(d0 - d1) > 0.02)
						pick = d0 > d1 ? 0 : 1;
					else if (have_last_raw)
						pick = dist2(cand[0], last_raw) <=
							       dist2(cand[1],
								     last_raw)
							       ? 0
							       : 1;
					else
						pick = by_err;
					if (pick != by_err)
						flip_picks++;
					// Learn the tag's world normal from
					// unambiguous frames only (off-frontal
					// views separate the two solutions).
					if (hi > lo * 3.0) {
						normal_ref.x +=
							(n_world[pick].x -
							 normal_ref.x) *
							0.1;
						normal_ref.y +=
							(n_world[pick].y -
							 normal_ref.y) *
							0.1;
						normal_ref.z +=
							(n_world[pick].z -
							 normal_ref.z) *
							0.1;
						have_normal_ref = true;
					}
				} else if (n_cand == 1 && !have_normal_ref) {
					normal_ref = n_world[0];
					have_normal_ref = true;
				}
				if (n_cand > 0) {
					if (have_last_raw)
						max_step_m = std::max(
							max_step_m,
							std::sqrt(dist2(
								cand[pick],
								last_raw)));
					acc_sum.x += cand[pick].x;
					acc_sum.y += cand[pick].y;
					acc_sum.z += cand[pick].z;
					acc_sum2.x += cand[pick].x * cand[pick].x;
					acc_sum2.y += cand[pick].y * cand[pick].y;
					acc_sum2.z += cand[pick].z * cand[pick].z;
					acc_n++;
					publish_marker_position(f, cand[pick],
								now_us32());
					last_raw = cand[pick];
					have_last_raw = true;
					last_pos = cand[pick];
					last_z = cand_z[pick];
				}
				if (pose1.R)
					matd_destroy(pose1.R);
				if (pose1.t)
					matd_destroy(pose1.t);
				if (pose2.R)
					matd_destroy(pose2.R);
				if (pose2.t)
					matd_destroy(pose2.t);
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
				vec3d sd = {};
				if (acc_n > 1) {
					const double n = acc_n;
					sd.x = std::sqrt(std::max(
						0.0, acc_sum2.x / n -
							     (acc_sum.x / n) *
								     (acc_sum.x / n)));
					sd.y = std::sqrt(std::max(
						0.0, acc_sum2.y / n -
							     (acc_sum.y / n) *
								     (acc_sum.y / n)));
					sd.z = std::sqrt(std::max(
						0.0, acc_sum2.z / n -
							     (acc_sum.z / n) *
								     (acc_sum.z / n)));
				}
				blog(LOG_INFO,
				     "[obs-nyan-real-3dof] marker 6DoF rate %u/%u range=%.3f raw=(%+.3f,%+.3f,%+.3f) head=(%+.3f,%+.3f,%+.3f)m raw_sd=(%.1f,%.1f,%.1f)mm step_max=%.1fmm flips=%u%s",
				     hits, frames, last_z, last_pos.x,
				     last_pos.y, last_pos.z, head.x, head.y,
				     head.z, sd.x * 1000.0, sd.y * 1000.0,
				     sd.z * 1000.0, max_step_m * 1000.0,
				     flip_picks,
				     head_valid ? "" : " (no anchor)");
				frames = 0;
				hits = 0;
				acc_sum = {};
				acc_sum2 = {};
				acc_n = 0;
				flip_picks = 0;
				max_step_m = 0.0;
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
