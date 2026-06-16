// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
#include "virtual_source.h"

#include <obs-module.h>
#include <graphics/graphics.h>
#include <graphics/vec2.h>
#include <graphics/vec3.h>
#include <graphics/vec4.h>
#include <util/platform.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "audio-wall-source.h" // nyan_real_audio_wall_active (j warning)
#include "device_manager.h"
#include "device_registry.h"
#include "display-wall-source.h"
#include "endpoint_volume.h" // default_render_endpoint_id (j warning)
#include "math_util.h"
#include "nyan_types.h"
#include "overlay_text.h" // in-glasses status overlay rasterizer (shared)
#include "warp_params.h"

// GPU stage probes (debug logging only): D3D11 timestamp queries around the
// capture and warp draws. gs_timer_get_data busy-waits until the GPU passes
// the query, so results are read two frames later (parity double buffer)
// when they are guaranteed complete and the read returns instantly.
constexpr int GPU_PROBE_VIEWS = 6; // preview/program/projectors per frame

struct gpu_probe {
	gs_timer_t *timer = nullptr;
	gs_timer_range_t *range = nullptr;
	bool pending = false;
};

static bool probe_begin(gpu_probe &p)
{
	if (!p.timer)
		p.timer = gs_timer_create();
	if (!p.range)
		p.range = gs_timer_range_create();
	if (!p.timer || !p.range)
		return false;
	gs_timer_range_begin(p.range);
	gs_timer_begin(p.timer);
	return true;
}

static void probe_end(gpu_probe &p)
{
	gs_timer_end(p.timer);
	gs_timer_range_end(p.range);
	p.pending = true;
}

static bool probe_read_ms(gpu_probe &p, double *ms)
{
	if (!p.pending)
		return false;
	p.pending = false;
	uint64_t ticks = 0;
	uint64_t freq = 0;
	bool disjoint = false;
	if (!gs_timer_get_data(p.timer, &ticks))
		return false;
	if (!gs_timer_range_get_data(p.range, &disjoint, &freq) || disjoint ||
	    !freq)
		return false;
	*ms = static_cast<double>(ticks) * 1000.0 / static_cast<double>(freq);
	return true;
}

static void probe_free(gpu_probe &p)
{
	if (p.timer)
		gs_timer_destroy(p.timer);
	if (p.range)
		gs_timer_range_destroy(p.range);
	p.timer = nullptr;
	p.range = nullptr;
	p.pending = false;
}

struct nyan_real_virtual_source {
	obs_source_t *context = nullptr;
	obs_source_t *target = nullptr;
	gs_texrender_t *texrender = nullptr;
	gs_effect_t *effect = nullptr;
	gs_eparam_t *p_image = nullptr;
	gs_eparam_t *p_pose_q = nullptr;
	gs_eparam_t *p_pose_valid = nullptr;
	gs_eparam_t *p_tan_half_fov = nullptr;
	gs_eparam_t *p_screen_distance_m = nullptr;
	gs_eparam_t *p_screen_half_size_m = nullptr;
	gs_eparam_t *p_screen_curve = nullptr;
	gs_eparam_t *p_eye_pos_m = nullptr;
	gs_eparam_t *p_debug_tint = nullptr;
	gs_eparam_t *p_offscreen_dir = nullptr;
	gs_eparam_t *p_offscreen_intensity = nullptr;
	gs_eparam_t *p_offscreen_band = nullptr;
	gs_eparam_t *p_flat_fit = nullptr;
	// In-glasses status overlay (calibrating, etc.): texture cached from the
	// shared Qt rasterizer, rebuilt only when the message changes.
	gs_texture_t *overlay_tex = nullptr;
	uint32_t overlay_w = 0;
	uint32_t overlay_h = 0;
	std::string overlay_msg;
	// Throttled cache for the (j) "output == Windows default" warning: the
	// default-device query is too heavy to run every view.
	uint64_t last_audio_check_ns = 0;
	bool audio_conflict = false;
	std::string target_name;
	uint32_t output_width = 1920;
	uint32_t output_height = 1080;
	bool target_active_child = false;
	bool target_recursion_blocked = false;
	// One target capture per output frame: video_render runs once per view
	// (preview, program, projectors), but re-rendering the target into the
	// texrender for each of them only repeats identical work. Reset by
	// video_tick, set after a successful capture.
	bool captured_this_frame = false;
	float target_retry_timer_s = 0.0f;
	uint64_t last_render_log_ns = 0;
	// GPU profiling state (graphics thread only, active under debug log).
	gpu_probe probe_capture[2];
	gpu_probe probe_warp[2][GPU_PROBE_VIEWS];
	int probe_parity = 0;
	int warp_view_count = 0;
	bool probes_read_this_frame = false;
	double acc_capture_ms = 0.0;
	double acc_warp_ms = 0.0;
	uint32_t acc_frames = 0;
	uint32_t acc_views = 0;
	uint64_t last_gpu_log_ns = 0;
};

static const char *virtual_source_get_name(void *)
{
	return obs_module_text("source.name");
}

static void bind_warp_effect(gs_effect_t *effect, gs_eparam_t **p_image,
			     gs_eparam_t **p_pose_q, gs_eparam_t **p_pose_valid,
			     gs_eparam_t **p_tan_half_fov,
			     gs_eparam_t **p_screen_distance_m,
			     gs_eparam_t **p_screen_half_size_m,
			     gs_eparam_t **p_screen_curve,
			     gs_eparam_t **p_eye_pos_m,
			     gs_eparam_t **p_debug_tint)
{
	if (!effect)
		return;
	if (p_image)
		*p_image = gs_effect_get_param_by_name(effect, "image");
	*p_pose_q = gs_effect_get_param_by_name(effect, "pose_q");
	*p_pose_valid = gs_effect_get_param_by_name(effect, "pose_valid");
	*p_tan_half_fov = gs_effect_get_param_by_name(effect, "tan_half_fov");
	*p_screen_distance_m =
		gs_effect_get_param_by_name(effect, "screen_distance_m");
	*p_screen_half_size_m =
		gs_effect_get_param_by_name(effect, "screen_half_size_m");
	*p_screen_curve = gs_effect_get_param_by_name(effect, "screen_curve");
	*p_eye_pos_m = gs_effect_get_param_by_name(effect, "eye_pos_m");
	*p_debug_tint = gs_effect_get_param_by_name(effect, "debug_tint");
}

static gs_effect_t *create_warp_effect(gs_eparam_t **p_image,
				       gs_eparam_t **p_pose_q,
				       gs_eparam_t **p_pose_valid,
				       gs_eparam_t **p_tan_half_fov,
				       gs_eparam_t **p_screen_distance_m,
				       gs_eparam_t **p_screen_half_size_m,
				       gs_eparam_t **p_screen_curve,
				       gs_eparam_t **p_eye_pos_m,
				       gs_eparam_t **p_debug_tint)
{
	// The libobs effect parser has no #include, so build the effect string by
	// prepending the shared warp math (data/nyan_warp.hlsli, also used by the
	// standalone) to the effect wrapper. Single GPU source of truth.
	char *hlsli_path = obs_module_file("nyan_warp.hlsli");
	char *effect_path = obs_module_file("nyan-real-3dof.effect");
	char *hlsli = hlsli_path ? os_quick_read_utf8_file(hlsli_path) : nullptr;
	char *wrapper = effect_path ? os_quick_read_utf8_file(effect_path) : nullptr;
	gs_effect_t *effect = nullptr;
	if (hlsli && wrapper) {
		const std::string combined = std::string(hlsli) + "\n" + wrapper;
		effect = gs_effect_create(combined.c_str(), "nyan-real-3dof.effect",
					  nullptr);
	}
	bfree(hlsli_path);
	bfree(effect_path);
	bfree(hlsli);
	bfree(wrapper);
	bind_warp_effect(effect, p_image, p_pose_q, p_pose_valid, p_tan_half_fov,
			 p_screen_distance_m, p_screen_half_size_m, p_screen_curve,
			 p_eye_pos_m, p_debug_tint);
	return effect;
}

// Pushes the core-computed warp_params into the effect's uniforms. The math
// itself lives in core (compute_warp_params) so the OBS source and the
// standalone renderer cannot drift.
static void set_warp_effect_parameters(const nyan_real_virtual_source *s,
				       const warp_params &wp)
{
	struct vec4 pose_q;
	pose_q.x = wp.pose_q[0];
	pose_q.y = wp.pose_q[1];
	pose_q.z = wp.pose_q[2];
	pose_q.w = wp.pose_q[3];
	gs_effect_set_vec4(s->p_pose_q, &pose_q);
	gs_effect_set_float(s->p_pose_valid, wp.pose_valid);
	struct vec2 tan_half_fov;
	tan_half_fov.x = wp.tan_half_fov[0];
	tan_half_fov.y = wp.tan_half_fov[1];
	gs_effect_set_vec2(s->p_tan_half_fov, &tan_half_fov);
	gs_effect_set_float(s->p_screen_distance_m, wp.screen_distance_m);
	struct vec2 screen_half_size_m;
	screen_half_size_m.x = wp.screen_half_size_m[0];
	screen_half_size_m.y = wp.screen_half_size_m[1];
	gs_effect_set_vec2(s->p_screen_half_size_m, &screen_half_size_m);
	gs_effect_set_float(s->p_screen_curve, wp.screen_curve);
	gs_effect_set_float(s->p_debug_tint, wp.debug_tint);
	struct vec2 offscreen_dir;
	offscreen_dir.x = wp.offscreen_dir[0];
	offscreen_dir.y = wp.offscreen_dir[1];
	gs_effect_set_vec2(s->p_offscreen_dir, &offscreen_dir);
	gs_effect_set_float(s->p_offscreen_intensity, wp.offscreen_intensity);
	struct vec2 offscreen_band;
	offscreen_band.x = wp.offscreen_band[0];
	offscreen_band.y = wp.offscreen_band[1];
	gs_effect_set_vec2(s->p_offscreen_band, &offscreen_band);
	struct vec2 flat_fit;
	flat_fit.x = wp.flat_fit[0];
	flat_fit.y = wp.flat_fit[1];
	gs_effect_set_vec2(s->p_flat_fit, &flat_fit);
}

struct recursion_check_data {
	obs_source_t *needle = nullptr;
	bool found = false;
};

static void check_source_recursion(obs_source_t *parent, obs_source_t *child,
				   void *param)
{
	auto *d = static_cast<recursion_check_data *>(param);
	if (parent == d->needle || child == d->needle)
		d->found = true;
}

static bool virtual_target_allowed(const nyan_real_virtual_source *s,
				   obs_source_t *candidate)
{
	if (!candidate || obs_source_removed(candidate) || candidate == s->context)
		return false;
	if ((obs_source_get_output_flags(candidate) & OBS_SOURCE_VIDEO) == 0)
		return false;
	// Referencing another virtual screen is almost always an accidental
	// double-warp or a recursion path through a scene.
	if (is_virtual_source_id(obs_source_get_id(candidate)))
		return false;
	recursion_check_data check;
	check.needle = s->context;
	obs_source_enum_full_tree(candidate, check_source_recursion, &check);
	if (check.found)
		return false;
	return true;
}

static void virtual_source_remove_active_child(nyan_real_virtual_source *s)
{
	if (s->target && s->target_active_child) {
		obs_source_remove_active_child(s->context, s->target);
		s->target_active_child = false;
	}
}

static bool virtual_source_add_active_child(nyan_real_virtual_source *s)
{
	if (!s->target || s->target_active_child)
		return true;
	if (!obs_source_showing(s->context)) {
		s->target_recursion_blocked = false;
		return true;
	}

	if (!obs_source_add_active_child(s->context, s->target)) {
		s->target_recursion_blocked = true;
		blog(LOG_WARNING,
		     "[obs-nyan-real-3dof] virtual screen target rejected to avoid recursive rendering: %s",
		     obs_source_get_name(s->target));
		return false;
	}

	s->target_active_child = true;
	s->target_recursion_blocked = false;
	return true;
}

static void virtual_source_release_target(nyan_real_virtual_source *s)
{
	virtual_source_remove_active_child(s);
	if (s->target)
		obs_source_release(s->target);
	s->target = nullptr;
	s->target_recursion_blocked = false;
}

static void virtual_source_set_target(nyan_real_virtual_source *s,
				      const char *target_name,
				      bool log_failure = true)
{
	obs_source_t *next = nullptr;
	if (target_name && *target_name)
		next = obs_get_source_by_name(target_name);
	if (next && !virtual_target_allowed(s, next)) {
		obs_source_release(next);
		next = nullptr;
	}

	if (next == s->target) {
		if (next)
			obs_source_release(next);
		virtual_source_add_active_child(s);
		return;
	}

	virtual_source_release_target(s);
	s->target = next;
	if (s->target) {
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] virtual screen target set: '%s' (%ux%u)",
		     obs_source_get_name(s->target),
		     obs_source_get_width(s->target),
		     obs_source_get_height(s->target));
		virtual_source_add_active_child(s);
	} else if (log_failure && target_name && *target_name) {
		blog(LOG_WARNING,
		     "[obs-nyan-real-3dof] virtual screen target was not usable: '%s'",
		     target_name);
	}
}

static void virtual_source_update(void *data, obs_data_t *settings);

static void *virtual_source_create(obs_data_t *settings, obs_source_t *context)
{
	auto *s = new nyan_real_virtual_source();
	s->context = context;

	obs_enter_graphics();
	s->effect = create_warp_effect(&s->p_image, &s->p_pose_q, &s->p_pose_valid,
				       &s->p_tan_half_fov,
				       &s->p_screen_distance_m,
				       &s->p_screen_half_size_m,
				       &s->p_screen_curve,
				       &s->p_eye_pos_m,
				       &s->p_debug_tint);
	if (s->effect) {
		s->p_offscreen_dir =
			gs_effect_get_param_by_name(s->effect, "offscreen_dir");
		s->p_offscreen_intensity = gs_effect_get_param_by_name(
			s->effect, "offscreen_intensity");
		s->p_offscreen_band =
			gs_effect_get_param_by_name(s->effect, "offscreen_band");
		s->p_flat_fit =
			gs_effect_get_param_by_name(s->effect, "flat_fit");
	}
	obs_leave_graphics();

	if (!s->effect) {
		blog(LOG_ERROR,
		     "[obs-nyan-real-3dof] nyan-real-3dof.effect missing -> virtual source disabled");
		delete s;
		return nullptr;
	}

	virtual_source_update(s, settings);
	g_device.virtual_source_count.fetch_add(1, std::memory_order_relaxed);
	blog(LOG_INFO, "[obs-nyan-real-3dof] virtual screen source created: %s",
	     BUILD_INFO);
	return s;
}

static void virtual_source_update(void *data, obs_data_t *settings)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	if (!settings) {
		s->target_name.clear();
		virtual_source_set_target(s, "");
		return;
	}
	const char *target_name = obs_data_get_string(settings, "target");
	s->target_name = target_name ? target_name : "";
	virtual_source_set_target(s, s->target_name.c_str());
	s->target_retry_timer_s = 0.0f;
}

static void virtual_source_destroy(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	virtual_source_release_target(s);
	obs_enter_graphics();
	if (s->texrender)
		gs_texrender_destroy(s->texrender);
	if (s->overlay_tex)
		gs_texture_destroy(s->overlay_tex);
	if (s->effect)
		gs_effect_destroy(s->effect);
	for (int parity = 0; parity < 2; parity++) {
		probe_free(s->probe_capture[parity]);
		for (gpu_probe &p : s->probe_warp[parity])
			probe_free(p);
	}
	obs_leave_graphics();
	g_device.virtual_source_count.fetch_sub(1, std::memory_order_relaxed);
	delete s;
}

static uint32_t virtual_source_get_width(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	return s->output_width;
}

static uint32_t virtual_source_get_height(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	return s->output_height;
}

static void virtual_source_defaults(obs_data_t *settings)
{
	obs_data_set_default_string(settings, "target", "");
}

struct source_list_data {
	obs_property_t *list = nullptr;
	obs_source_t *self = nullptr;
	std::vector<std::string> names;
};

static bool add_source_to_property_list(void *data, obs_source_t *source)
{
	auto *d = static_cast<source_list_data *>(data);
	if (!source || source == d->self || obs_source_removed(source))
		return true;
	if ((obs_source_get_output_flags(source) & OBS_SOURCE_VIDEO) == 0)
		return true;
	if (is_virtual_source_id(obs_source_get_id(source)))
		return true;

	const char *name = obs_source_get_name(source);
	if (!name || !*name)
		return true;
	if (std::find(d->names.begin(), d->names.end(), name) != d->names.end())
		return true;
	d->names.emplace_back(name);
	return true;
}

static obs_properties_t *virtual_source_properties(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	obs_properties_t *props = obs_properties_create();
	obs_properties_add_text(props, "build_info", BUILD_INFO, OBS_TEXT_INFO);
	obs_properties_add_text(props, "source_global_notice",
				obs_module_text("source_global_notice"),
				OBS_TEXT_INFO);

	std::string render_notice =
		obs_module_text("source.render_resolution_notice");
	if (s) {
		render_notice += "\n";
		render_notice += std::to_string(s->output_width);
		render_notice += " x ";
		render_notice += std::to_string(s->output_height);
		render_notice += " px";
	}
	obs_properties_add_text(props, "source_render_resolution_notice",
				render_notice.c_str(), OBS_TEXT_INFO);
	obs_property_t *target = obs_properties_add_list(
		props, "target", obs_module_text("source.target"),
		OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);
	obs_property_list_add_string(target, obs_module_text("source.target.none"),
				     "");

	source_list_data list_data;
	list_data.list = target;
	list_data.self = s ? s->context : nullptr;
	obs_enum_scenes(add_source_to_property_list, &list_data);
	obs_enum_sources(add_source_to_property_list, &list_data);
	std::sort(list_data.names.begin(), list_data.names.end());
	for (const auto &name : list_data.names)
		obs_property_list_add_string(target, name.c_str(), name.c_str());

	std::string target_summary = obs_module_text("source.target_summary_none");
	if (s && s->target && !obs_source_removed(s->target)) {
		target_summary = obs_module_text("source.target_summary_prefix");
		target_summary += obs_source_get_name(s->target);
		target_summary += " (";
		target_summary += std::to_string(obs_source_get_width(s->target));
		target_summary += " x ";
		target_summary += std::to_string(obs_source_get_height(s->target));
		target_summary += " px)";
	}
	obs_properties_add_text(props, "source_target_summary",
				target_summary.c_str(), OBS_TEXT_INFO);
	return props;
}

static bool virtual_source_capture_target(nyan_real_virtual_source *s, uint32_t w,
					  uint32_t h,
					  enum gs_color_space space)
{
	const enum gs_color_format format = gs_get_format_from_space(space);
	if (s->texrender && gs_texrender_get_format(s->texrender) != format) {
		gs_texrender_destroy(s->texrender);
		s->texrender = nullptr;
	}
	if (!s->texrender)
		s->texrender = gs_texrender_create(format, GS_ZS_NONE);
	if (!s->texrender)
		return false;

	gs_texrender_reset(s->texrender);
	if (!gs_texrender_begin_with_color_space(s->texrender, w, h, space))
		return false;

	struct vec4 clear_color;
	vec4_zero(&clear_color);
	gs_clear(GS_CLEAR_COLOR, &clear_color, 0.0f, 0);
	gs_ortho(0.0f, static_cast<float>(w), 0.0f, static_cast<float>(h),
		 -100.0f, 100.0f);

	gs_blend_state_push();
	gs_blend_function_separate(GS_BLEND_SRCALPHA, GS_BLEND_INVSRCALPHA,
				   GS_BLEND_ONE, GS_BLEND_INVSRCALPHA);
	obs_source_video_render(s->target);
	gs_blend_state_pop();

	gs_texrender_end(s->texrender);
	return true;
}

// Rebuild the overlay texture when the message changes (graphics context).
static void virtual_source_set_overlay(nyan_real_virtual_source *s,
				       const std::string &msg, int font_px)
{
	if (msg == s->overlay_msg)
		return;
	s->overlay_msg = msg;
	if (s->overlay_tex) {
		gs_texture_destroy(s->overlay_tex);
		s->overlay_tex = nullptr;
	}
	s->overlay_w = s->overlay_h = 0;
	if (msg.empty())
		return;
	const overlay_bitmap bmp = overlay_text_rasterize(msg, font_px);
	if (bmp.width <= 0 || bmp.height <= 0)
		return;
	const uint8_t *data = bmp.rgba.data();
	s->overlay_tex = gs_texture_create(static_cast<uint32_t>(bmp.width),
					   static_cast<uint32_t>(bmp.height),
					   GS_RGBA, 1, &data, 0);
	if (s->overlay_tex) {
		s->overlay_w = static_cast<uint32_t>(bmp.width);
		s->overlay_h = static_cast<uint32_t>(bmp.height);
	}
}

// Composite the overlay centered over the warped output (per eye for SBS), with
// straight-alpha blending (the Qt bitmap is non-premultiplied). conv_px shifts
// the left eye's copy right and the right eye's left so a 2D card converges at
// the virtual screen's depth (0 = none). Uses the warp's translate + sized
// gs_draw_sprite idiom (no scale matrix) so both eyes draw reliably.
static void virtual_source_draw_overlay(nyan_real_virtual_source *s, bool sbs,
					uint32_t eye_w, int conv_px)
{
	if (!s->overlay_tex || !s->overlay_w || !s->overlay_h)
		return;
	gs_effect_t *eff = obs_get_base_effect(OBS_EFFECT_DEFAULT);
	gs_effect_set_texture(gs_effect_get_param_by_name(eff, "image"),
			      s->overlay_tex);
	gs_blend_state_push();
	gs_blend_function(GS_BLEND_SRCALPHA, GS_BLEND_INVSRCALPHA);
	const uint32_t region_w = sbs ? eye_w : s->output_width;
	const int eye_count = sbs ? 2 : 1;
	// Both eyes are drawn inside one effect pass (as the warp does); looping
	// the effect per eye left the second eye undrawn.
	while (gs_effect_loop(eff, "Draw")) {
		for (int e = 0; e < eye_count; e++) {
			float scale = 1.0f;
			const float maxw = static_cast<float>(region_w) * 0.9f;
			if (static_cast<float>(s->overlay_w) > maxw)
				scale = maxw / static_cast<float>(s->overlay_w);
			const uint32_t dw = static_cast<uint32_t>(
				static_cast<float>(s->overlay_w) * scale + 0.5f);
			const uint32_t dh = static_cast<uint32_t>(
				static_cast<float>(s->overlay_h) * scale + 0.5f);
			const float sign = (e == 0) ? 1.0f : -1.0f;
			const float ox =
				static_cast<float>(region_w) *
					static_cast<float>(e) +
				(static_cast<float>(region_w) -
				 static_cast<float>(dw)) /
					2.0f +
				sign * static_cast<float>(conv_px);
			const float oy = (static_cast<float>(s->output_height) -
					  static_cast<float>(dh)) /
					 2.0f;
			gs_matrix_push();
			gs_matrix_translate3f(ox, oy, 0.0f);
			gs_draw_sprite(s->overlay_tex, 0, dw, dh);
			gs_matrix_pop();
		}
	}
	gs_blend_state_pop();
}

static void virtual_source_draw_warp(nyan_real_virtual_source *s, gs_texture_t *tex,
				     uint32_t source_w, uint32_t source_h)
{
	// All warp math is shared in core (warp_params) so the OBS source and the
	// standalone renderer cannot drift.
	const eye_layout eyes =
		compute_eye_layout(s->output_width, s->output_height);
	const warp_params wp = compute_warp_params(
		eyes, s->output_height, source_w, source_h,
		hid_device_ready(&g_device), nyan_real_wall_center_u());
	set_warp_effect_parameters(s, wp);

	const bool sbs = eyes.sbs;
	const uint32_t eye_w = eyes.eye_w;

	// Per-eye ray origin = viewer + sign*eye_right (gaze-dolly viewer offset +
	// IPD parallax; sign -1 = left/mono, +1 = right).
	struct vec3 eye_pos;
	vec3_set(&eye_pos, static_cast<float>(wp.viewer.x - wp.eye_right.x),
		 static_cast<float>(wp.viewer.y - wp.eye_right.y),
		 static_cast<float>(wp.viewer.z - wp.eye_right.z));
	gs_effect_set_vec3(s->p_eye_pos_m, &eye_pos); // left eye (or mono center)

	const bool previous_srgb = gs_set_linear_srgb(true);
	const bool linear_srgb = gs_get_linear_srgb();
	const bool previous_fb = gs_framebuffer_srgb_enabled();
	gs_enable_framebuffer_srgb(linear_srgb);
	if (linear_srgb)
		gs_effect_set_texture_srgb(s->p_image, tex);
	else
		gs_effect_set_texture(s->p_image, tex);

	gs_technique_t *tech = gs_effect_get_technique(s->effect, "Draw");
	const size_t passes = gs_technique_begin(tech);
	for (size_t i = 0; i < passes; i++) {
		gs_technique_begin_pass(tech, i);
		gs_draw_sprite(tex, 0, eye_w, s->output_height);
		if (sbs) {
			// Effect parameters set between draws are flushed by
			// device_draw via gs_effect_update_params, so the
			// right half picks up the right-eye origin.
			vec3_set(&eye_pos,
				 static_cast<float>(wp.viewer.x + wp.eye_right.x),
				 static_cast<float>(wp.viewer.y + wp.eye_right.y),
				 static_cast<float>(wp.viewer.z + wp.eye_right.z));
			gs_effect_set_vec3(s->p_eye_pos_m, &eye_pos);
			gs_matrix_push();
			gs_matrix_translate3f(static_cast<float>(eye_w), 0.0f,
					      0.0f);
			gs_draw_sprite(tex, 0, eye_w, s->output_height);
			gs_matrix_pop();
		}
		gs_technique_end_pass(tech);
	}
	gs_technique_end(tech);
	gs_enable_framebuffer_srgb(previous_fb);
	gs_set_linear_srgb(previous_srgb);

	// In-glasses status overlay centered over the warp. Calibrating (transient)
	// wins over the audio-conflict warning (Audio Wall on and its monitoring
	// output is the Windows default = raw + spatialized double up). The
	// default-device query is throttled to ~1 Hz.
	bool calibrated;
	{
		std::lock_guard<std::mutex> lk(g_device.state_mutex);
		calibrated = g_device.pose.calibrated;
	}
	const uint64_t now = os_gettime_ns();
	if (now - s->last_audio_check_ns > 1000000000ULL) {
		s->last_audio_check_ns = now;
		s->audio_conflict = false;
		if (nyan_real_audio_wall_active()) {
			const char *mon_name = nullptr;
			const char *mon_id = nullptr;
			obs_get_audio_monitoring_device(&mon_name, &mon_id);
			const std::string id = mon_id ? mon_id : "";
			s->audio_conflict =
				id == "default" ||
				(!id.empty() &&
				 id == default_render_endpoint_id());
		}
	}
	std::string ov;
	// Glasses-only: with nothing plugged in there's no in-glasses view worth
	// annotating, and "@auto" audio falls back to the default endpoint, which
	// would otherwise false-trigger the audio_default notice forever (it only
	// matters once audio is actually routed to the glasses).
	if (g_device.connected.load(std::memory_order_relaxed)) {
		if (!calibrated)
			ov = obs_module_text("overlay.calibrating");
		else if (s->audio_conflict)
			ov = obs_module_text("overlay.audio_default");
	}
	virtual_source_set_overlay(s, ov,
				   static_cast<int>(s->output_height / 16));
	// Converge the 2D card to the virtual screen's depth (per-eye disparity);
	// mono has no disparity. Shared projection with the standalone backend.
	const int conv_px =
		sbs ? overlay_convergence_px(
			      eye_w, wp.tan_half_fov[0], wp.screen_distance_m,
			      g_device.ipd_mm.load(std::memory_order_relaxed) /
				      1000.0)
		    : 0;
	virtual_source_draw_overlay(s, sbs, eye_w, conv_px);
}

// Read the probes written two frames ago and emit a once-a-second summary.
// Runs inside the graphics context on the first view render of the frame.
static void gpu_probes_collect(nyan_real_virtual_source *s)
{
	double ms = 0.0;
	if (probe_read_ms(s->probe_capture[s->probe_parity], &ms)) {
		s->acc_capture_ms += ms;
		s->acc_frames++;
	}
	for (gpu_probe &p : s->probe_warp[s->probe_parity]) {
		if (probe_read_ms(p, &ms)) {
			s->acc_warp_ms += ms;
			s->acc_views++;
		}
	}

	const uint64_t now = os_gettime_ns();
	if (now - s->last_gpu_log_ns < 1000000000ULL || !s->acc_frames)
		return;
	s->last_gpu_log_ns = now;
	const double frames = static_cast<double>(s->acc_frames);
	blog(LOG_INFO,
	     "[obs-nyan-real-3dof] GPU: capture %.2f ms/frame, warp %.2f ms/frame (%.1f views, %.2f ms/view), output %ux%u, %u frames",
	     s->acc_capture_ms / frames, s->acc_warp_ms / frames,
	     static_cast<double>(s->acc_views) / frames,
	     s->acc_views ? s->acc_warp_ms / s->acc_views : 0.0,
	     s->output_width, s->output_height, s->acc_frames);
	s->acc_capture_ms = 0.0;
	s->acc_warp_ms = 0.0;
	s->acc_frames = 0;
	s->acc_views = 0;
}

static void virtual_source_render(void *data, gs_effect_t *)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	if (!s->effect || !s->p_image || !s->target || s->target_recursion_blocked ||
	    obs_source_removed(s->target))
		return;

	const bool gpu_debug = g_device.debug_log.load(std::memory_order_relaxed);
	if (gpu_debug && !s->probes_read_this_frame) {
		s->probes_read_this_frame = true;
		gpu_probes_collect(s);
	}

	const uint32_t source_w = obs_source_get_width(s->target);
	const uint32_t source_h = obs_source_get_height(s->target);
	if (source_w == 0 || source_h == 0 || s->output_width == 0 ||
	    s->output_height == 0)
		return;

	const enum gs_color_space pref[] = {GS_CS_SRGB};
	const enum gs_color_space space =
		obs_source_get_color_space(s->target, 1, pref);
	if (!s->captured_this_frame) {
		gpu_probe &cp = s->probe_capture[s->probe_parity];
		const bool probing = gpu_debug && probe_begin(cp);
		const bool captured =
			virtual_source_capture_target(s, source_w, source_h,
						      space);
		if (probing)
			probe_end(cp);
		if (!captured) {
			const uint64_t now = os_gettime_ns();
			if (now - s->last_render_log_ns > 2000000000ULL) {
				s->last_render_log_ns = now;
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] virtual screen capture failed: target='%s' size=%ux%u",
				     obs_source_get_name(s->target), source_w,
				     source_h);
			}
			return;
		}
		s->captured_this_frame = true;
	}

	gs_texture_t *tex = gs_texrender_get_texture(s->texrender);
	if (!tex) {
		const uint64_t now = os_gettime_ns();
		if (now - s->last_render_log_ns > 2000000000ULL) {
			s->last_render_log_ns = now;
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] virtual screen texture was unavailable");
		}
		return;
	}

	gs_blend_state_push();
	gs_blend_function(GS_BLEND_ONE, GS_BLEND_INVSRCALPHA);
	gpu_probe *wp = gpu_debug && s->warp_view_count < GPU_PROBE_VIEWS
				? &s->probe_warp[s->probe_parity]
						[s->warp_view_count]
				: nullptr;
	if (wp && !probe_begin(*wp))
		wp = nullptr;
	virtual_source_draw_warp(s, tex, source_w, source_h);
	if (wp)
		probe_end(*wp);
	s->warp_view_count++;
	gs_blend_state_pop();
}

static void virtual_source_tick(void *data, float seconds)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	if (!s)
		return;
	s->captured_this_frame = false;
	// GPU probes: this frame writes the other parity slot; what that slot
	// held (two frames old) is read at the first view render.
	s->probe_parity ^= 1;
	s->warp_view_count = 0;
	s->probes_read_this_frame = false;

	// Render resolution is automatic: the glasses display's actual mode
	// when present, otherwise the HID-detected device's native resolution.
	uint32_t auto_w = g_glasses_display_width.load(std::memory_order_relaxed);
	uint32_t auto_h =
		g_glasses_display_height.load(std::memory_order_relaxed);
	if (!auto_w || !auto_h) {
		const model_profile &profile =
			profile_for(detected_hid_model(&g_device));
		auto_w = profile.display_width;
		auto_h = profile.display_height;
	}
	s->output_width = auto_w;
	s->output_height = auto_h;

	if (s->target_name.empty())
		return;

	if (s->target && obs_source_removed(s->target))
		virtual_source_release_target(s);

	if (s->target) {
		virtual_source_add_active_child(s);
		return;
	}

	s->target_retry_timer_s += seconds;
	if (s->target_retry_timer_s < VIRTUAL_TARGET_RETRY_INTERVAL_S)
		return;

	s->target_retry_timer_s = 0.0f;
	virtual_source_set_target(s, s->target_name.c_str(), false);
}

static void virtual_source_show(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	virtual_source_add_active_child(s);
	s->target_retry_timer_s = VIRTUAL_TARGET_RETRY_INTERVAL_S;
}

static void virtual_source_hide(void *data)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	virtual_source_remove_active_child(s);
	s->target_recursion_blocked = false;
}

static void virtual_source_enum_active(void *data,
				       obs_source_enum_proc_t enum_callback,
				       void *param)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	if (s->target && !s->target_recursion_blocked)
		enum_callback(s->context, s->target, param);
}

static bool virtual_source_audio_render(void *, uint64_t *,
					obs_source_audio_mix *, uint32_t, size_t,
					size_t)
{
	return false;
}

static enum gs_color_space virtual_source_get_color_space(void *, size_t,
						     const enum gs_color_space *)
{
	return GS_CS_SRGB;
}

static obs_source_info nyan_real_3dof_virtual_info = {};

void register_nyan_real_virtual_source()
{
	nyan_real_3dof_virtual_info.id = "nyan_real_3dof_virtual_screen";
	nyan_real_3dof_virtual_info.type = OBS_SOURCE_TYPE_INPUT;
	nyan_real_3dof_virtual_info.output_flags =
		OBS_SOURCE_VIDEO | OBS_SOURCE_CUSTOM_DRAW | OBS_SOURCE_COMPOSITE |
		OBS_SOURCE_SRGB | OBS_SOURCE_DO_NOT_DUPLICATE;
	nyan_real_3dof_virtual_info.get_name = virtual_source_get_name;
	nyan_real_3dof_virtual_info.create = virtual_source_create;
	nyan_real_3dof_virtual_info.destroy = virtual_source_destroy;
	nyan_real_3dof_virtual_info.get_width = virtual_source_get_width;
	nyan_real_3dof_virtual_info.get_height = virtual_source_get_height;
	nyan_real_3dof_virtual_info.get_defaults = virtual_source_defaults;
	nyan_real_3dof_virtual_info.get_properties = virtual_source_properties;
	nyan_real_3dof_virtual_info.update = virtual_source_update;
	nyan_real_3dof_virtual_info.video_render = virtual_source_render;
	nyan_real_3dof_virtual_info.video_tick = virtual_source_tick;
	nyan_real_3dof_virtual_info.show = virtual_source_show;
	nyan_real_3dof_virtual_info.hide = virtual_source_hide;
	nyan_real_3dof_virtual_info.enum_active_sources = virtual_source_enum_active;
	nyan_real_3dof_virtual_info.enum_all_sources = virtual_source_enum_active;
	nyan_real_3dof_virtual_info.audio_render = virtual_source_audio_render;
	nyan_real_3dof_virtual_info.video_get_color_space =
		virtual_source_get_color_space;
	nyan_real_3dof_virtual_info.icon_type = OBS_ICON_TYPE_CUSTOM;
	obs_register_source(&nyan_real_3dof_virtual_info);
}
