// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Audio filter that anchors a source at a fixed bearing in the virtual space
// and pans it against the shared head pose, so the sound of "the monitor on
// the left" keeps coming from the left as the head turns. v1 is a constant-
// power panner with optional screen-distance attenuation; ITD/HRTF can be
// layered on later without changing the filter's interface.
#include <obs-module.h>
#include <media-io/audio-io.h>
#include <util/platform.h>

#include <atomic>
#include <cmath>
#include <mutex>

#include "device_manager.h"
#include "math_util.h"
#include "nyan_types.h"
#include "spatial_audio_filter.h"
#include "spatial_pan.h"

namespace {

struct nyan_spatial_filter {
	obs_source_t *context = nullptr;
	// Written by update() on the UI thread, read by the audio thread.
	std::atomic<float> azimuth_deg{0.0f};
	std::atomic<int> mode{SPATIAL_MODE_POINT};
	std::atomic<bool> distance_gain{true};
	// Smoothed gains, audio thread only. Ramped linearly across each
	// block to avoid zipper noise on head motion.
	bool have_gains = false;
	float gl_cur = 0.0f; // left output (pan + behind + distance)
	float gr_cur = 0.0f; // right output
	float gx_cur = 0.0f; // non-panned channels (behind + distance only)
	uint64_t last_debug_ns = 0; // audio thread only
};

const char *spatial_get_name(void *)
{
	return obs_module_text("spatial.name");
}

void spatial_update(void *data, obs_data_t *settings)
{
	auto *s = static_cast<nyan_spatial_filter *>(data);
	s->azimuth_deg.store(
		static_cast<float>(obs_data_get_double(settings, "azimuth_deg")),
		std::memory_order_relaxed);
	s->mode.store(static_cast<int>(obs_data_get_int(settings, "mode")),
		      std::memory_order_relaxed);
	s->distance_gain.store(obs_data_get_bool(settings, "distance_gain"),
			       std::memory_order_relaxed);
}

void *spatial_create(obs_data_t *settings, obs_source_t *context)
{
	auto *s = new nyan_spatial_filter();
	s->context = context;
	spatial_update(s, settings);
	return s;
}

// Spatialized audio is meant to be heard through OBS's monitoring path, so
// adding the filter flips the parent source to monitor-only automatically.
// Only an unset monitoring type is upgraded: an explicit user choice
// (monitor+output, or deliberately off after the filter was added) stays.
void spatial_filter_add(void *data, obs_source_t *parent)
{
	auto *s = static_cast<nyan_spatial_filter *>(data);
	if (!parent ||
	    obs_source_get_monitoring_type(parent) != OBS_MONITORING_TYPE_NONE)
		return;
	obs_source_set_monitoring_type(parent, OBS_MONITORING_TYPE_MONITOR_ONLY);
	blog(LOG_INFO,
	     "[obs-nyan-real-3dof] spatial '%s': set '%s' to monitor-only",
	     obs_source_get_name(s->context), obs_source_get_name(parent));
}

void spatial_destroy(void *data)
{
	delete static_cast<nyan_spatial_filter *>(data);
}

void spatial_defaults(obs_data_t *settings)
{
	obs_data_set_default_double(settings, "azimuth_deg", 0.0);
	obs_data_set_default_int(settings, "mode", SPATIAL_MODE_POINT);
	obs_data_set_default_bool(settings, "distance_gain", true);
}

obs_properties_t *spatial_properties(void *)
{
	obs_properties_t *props = obs_properties_create();
	obs_properties_add_text(props, "spatial_notice",
				obs_module_text("spatial.notice"), OBS_TEXT_INFO);
	obs_property_t *az = obs_properties_add_float_slider(
		props, "azimuth_deg", obs_module_text("spatial.azimuth"), -90.0,
		90.0, 1.0);
	obs_property_set_long_description(
		az, obs_module_text("spatial.azimuth_tooltip"));
	obs_property_t *mode = obs_properties_add_list(
		props, "mode", obs_module_text("spatial.mode"),
		OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
	obs_property_list_add_int(mode, obs_module_text("spatial.mode.point"),
				  SPATIAL_MODE_POINT);
	obs_property_list_add_int(mode, obs_module_text("spatial.mode.balance"),
				  SPATIAL_MODE_BALANCE);
	obs_property_t *dist = obs_properties_add_bool(
		props, "distance_gain",
		obs_module_text("spatial.distance_gain"));
	obs_property_set_long_description(
		dist, obs_module_text("spatial.distance_gain_tooltip"));
	return props;
}

obs_audio_data *spatial_filter_audio(void *data, obs_audio_data *audio)
{
	auto *s = static_cast<nyan_spatial_filter *>(data);
	if (!audio || !audio->frames || !audio->data[0])
		return audio;

	const double yaw = spatial_current_head_yaw();
	const int mode = s->mode.load(std::memory_order_relaxed);
	double rel = 0.0;
	const spatial_gains g = compute_spatial_gains(
		clampd(s->azimuth_deg.load(std::memory_order_relaxed), -90.0,
		       90.0),
		yaw, mode, s->distance_gain.load(std::memory_order_relaxed),
		&rel);
	const float gl_t = g.gl;
	const float gr_t = g.gr;
	const float gx_t = g.gx;

	// Once a second under the dock's debug switch: proves the filter runs
	// and shows whether the pose yaw actually reaches the audio thread.
	if (g_device.debug_log.load(std::memory_order_relaxed)) {
		const uint64_t now = os_gettime_ns();
		if (now - s->last_debug_ns > 1000000000ULL) {
			s->last_debug_ns = now;
			blog(LOG_INFO,
			     "[obs-nyan-real-3dof] spatial '%s': yaw %.1f deg, rel %.1f deg, gains L %.2f R %.2f",
			     obs_source_get_name(s->context),
			     yaw * 180.0 / PI, rel * 180.0 / PI, gl_t, gr_t);
		}
	}

	if (!s->have_gains) {
		s->gl_cur = gl_t;
		s->gr_cur = gr_t;
		s->gx_cur = gx_t;
		s->have_gains = true;
	}
	const float gl0 = s->gl_cur;
	const float gr0 = s->gr_cur;
	const float gx0 = s->gx_cur;
	const uint32_t n = audio->frames;
	const float step = n > 1 ? 1.0f / static_cast<float>(n - 1) : 1.0f;

	float *l = reinterpret_cast<float *>(audio->data[0]);
	float *r = reinterpret_cast<float *>(audio->data[1]);
	if (l && r) {
		if (mode == SPATIAL_MODE_BALANCE) {
			for (uint32_t i = 0; i < n; i++) {
				const float t = i * step;
				l[i] *= gl0 + (gl_t - gl0) * t;
				r[i] *= gr0 + (gr_t - gr0) * t;
			}
		} else {
			// Point source: the screen is one sound source, so
			// collapse the internal stereo and pan the sum.
			for (uint32_t i = 0; i < n; i++) {
				const float t = i * step;
				const float m = 0.5f * (l[i] + r[i]);
				l[i] = m * (gl0 + (gl_t - gl0) * t);
				r[i] = m * (gr0 + (gr_t - gr0) * t);
			}
		}
	} else {
		// Mono source: panning needs two output channels the filter
		// does not have, so only the behind/distance level applies.
		for (uint32_t i = 0; i < n; i++)
			l[i] *= gx0 + (gx_t - gx0) * i * step;
	}
	for (size_t c = 2; c < MAX_AV_PLANES; c++) {
		float *x = reinterpret_cast<float *>(audio->data[c]);
		if (!x)
			continue;
		for (uint32_t i = 0; i < n; i++)
			x[i] *= gx0 + (gx_t - gx0) * i * step;
	}

	s->gl_cur = gl_t;
	s->gr_cur = gr_t;
	s->gx_cur = gx_t;
	return audio;
}

} // namespace

void register_nyan_real_spatial_audio_filter()
{
	static obs_source_info info = {};
	info.id = "nyan_real_3dof_spatial_audio";
	info.type = OBS_SOURCE_TYPE_FILTER;
	info.output_flags = OBS_SOURCE_AUDIO;
	info.get_name = spatial_get_name;
	info.create = spatial_create;
	info.destroy = spatial_destroy;
	info.get_defaults = spatial_defaults;
	info.get_properties = spatial_properties;
	info.update = spatial_update;
	info.filter_add = spatial_filter_add;
	info.filter_audio = spatial_filter_audio;
	obs_register_source(&info);
}
