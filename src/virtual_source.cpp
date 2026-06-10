// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
#include "virtual_source.h"

#include <obs-module.h>
#include <graphics/graphics.h>
#include <graphics/vec2.h>
#include <graphics/vec4.h>
#include <util/platform.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "device_manager.h"
#include "device_registry.h"
#include "math_util.h"
#include "nyan_types.h"

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
	gs_eparam_t *p_debug_tint = nullptr;
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
};

static quatd predict_pose(const pose_snapshot &p, float prediction_ms)
{
	quatd q = p.q;
	const double dt = clampd(prediction_ms, 0.0, 50.0) / 1000.0;
	const double wn =
		std::sqrt(p.omega.x * p.omega.x + p.omega.y * p.omega.y + p.omega.z * p.omega.z);
	if (p.calibrated && std::isfinite(wn) && wn > 1e-6 && dt > 0.0) {
		const double angle = wn * dt;
		const double h = 0.5 * angle;
		const double s = std::sin(h) / wn;
		const quatd dq = {std::cos(h), p.omega.x * s, p.omega.y * s,
				  p.omega.z * s};
		q = quat_normalize(quat_multiply(q, dq));
	}
	return q;
}

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
	*p_debug_tint = gs_effect_get_param_by_name(effect, "debug_tint");
}

static gs_effect_t *create_warp_effect(gs_eparam_t **p_image,
				       gs_eparam_t **p_pose_q,
				       gs_eparam_t **p_pose_valid,
				       gs_eparam_t **p_tan_half_fov,
				       gs_eparam_t **p_screen_distance_m,
				       gs_eparam_t **p_screen_half_size_m,
				       gs_eparam_t **p_screen_curve,
				       gs_eparam_t **p_debug_tint)
{
	char *effect_path = obs_module_file("nyan-real-3dof.effect");
	gs_effect_t *effect = gs_effect_create_from_file(effect_path, nullptr);
	bfree(effect_path);
	bind_warp_effect(effect, p_image, p_pose_q, p_pose_valid, p_tan_half_fov,
			 p_screen_distance_m, p_screen_half_size_m, p_screen_curve,
			 p_debug_tint);
	return effect;
}

static void set_warp_effect_parameters(gs_eparam_t *p_pose_q,
				       gs_eparam_t *p_pose_valid,
				       gs_eparam_t *p_tan_half_fov,
				       gs_eparam_t *p_screen_distance_m,
				       gs_eparam_t *p_screen_half_size_m,
				       gs_eparam_t *p_screen_curve,
				       gs_eparam_t *p_debug_tint,
				       uint32_t view_w, uint32_t view_h,
				       uint32_t screen_w, uint32_t screen_h,
				       bool enable_pose)
{
	pose_snapshot p;
	{
		std::lock_guard<std::mutex> lk(g_device.state_mutex);
		p = g_device.pose;
	}
	const quatd q = predict_pose(
		p, g_device.prediction_ms.load(std::memory_order_relaxed));
	struct vec4 pose_q;
	pose_q.x = static_cast<float>(q.w);
	pose_q.y = static_cast<float>(q.x);
	pose_q.z = static_cast<float>(q.y);
	pose_q.w = static_cast<float>(q.z);
	gs_effect_set_vec4(p_pose_q, &pose_q);
	gs_effect_set_float(p_pose_valid,
			    (enable_pose && p.calibrated && p.connected) ? 1.0f
									 : 0.0f);

	// The global FOV value is the single source of truth for rendering. When
	// auto FOV is on, the resolved HID model writes its FOV into this value;
	// otherwise the dock's manual value is used.
	const float diagonal_fov_deg = static_cast<float>(
		clampd(g_device.fov_deg.load(std::memory_order_relaxed), 20.0, 100.0));
	const float view_aspect = view_h ? static_cast<float>(view_w) /
						 static_cast<float>(view_h)
					: 1.0f;
	const float screen_aspect = screen_h ? static_cast<float>(screen_w) /
						   static_cast<float>(screen_h)
					      : view_aspect;
	const float screen_height_factor =
		(view_h > 0 && screen_h > view_h)
			? static_cast<float>(screen_h) / static_cast<float>(view_h)
			: 1.0f;
	/* XREAL's public FOV is conventionally diagonal. Treat the UI value the
	 * same way and derive the viewer's horizontal/vertical tangents from the
	 * output aspect. The physical virtual screen keeps at least that viewer
	 * height, expands vertically when the referenced texture is taller than the
	 * output view, and uses the referenced texture's aspect. This lets multi-row
	 * display walls extend vertically instead of being squeezed into one view. */
	const float tan_diag =
		std::tan(diagonal_fov_deg * static_cast<float>(PI) / 360.0f);
	const float diag_scale = std::sqrt(view_aspect * view_aspect + 1.0f);
	const float tan_x = tan_diag * view_aspect / diag_scale;
	const float tan_y = tan_diag / diag_scale;
	const float screen_distance_m = static_cast<float>(
		clampd(g_device.screen_distance_m.load(std::memory_order_relaxed), 1.0,
		       10.0));
	const float screen_size_factor = static_cast<float>(
		clampd(g_device.screen_size_factor.load(std::memory_order_relaxed), 0.25,
		       4.0));
	const float screen_curve = static_cast<float>(
		clampd(g_device.screen_curve.load(std::memory_order_relaxed), 0.0,
		       MAX_SCREEN_CURVE));
	struct vec2 tan_half_fov;
	tan_half_fov.x = tan_x;
	tan_half_fov.y = tan_y;
	struct vec2 screen_half_size_m;
	screen_half_size_m.y = DEFAULT_SCREEN_DISTANCE_M * tan_y *
			       screen_size_factor * screen_height_factor;
	screen_half_size_m.x = screen_half_size_m.y * screen_aspect;
	gs_effect_set_vec2(p_tan_half_fov, &tan_half_fov);
	gs_effect_set_float(p_screen_distance_m, screen_distance_m);
	gs_effect_set_vec2(p_screen_half_size_m, &screen_half_size_m);
	gs_effect_set_float(p_screen_curve, screen_curve);
	gs_effect_set_float(p_debug_tint,
			    g_device.debug_log.load(std::memory_order_relaxed)
				    ? (p.connected ? 0.25f : 0.6f)
				    : 0.0f);
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
				       &s->p_debug_tint);
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
	if (s->effect)
		gs_effect_destroy(s->effect);
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

static void virtual_source_draw_warp(nyan_real_virtual_source *s, gs_texture_t *tex,
				     uint32_t source_w, uint32_t source_h)
{
	set_warp_effect_parameters(s->p_pose_q, s->p_pose_valid, s->p_tan_half_fov,
				   s->p_screen_distance_m,
				   s->p_screen_half_size_m, s->p_screen_curve,
				   s->p_debug_tint,
				   s->output_width, s->output_height, source_w,
				   source_h, hid_device_ready(&g_device));

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
		gs_draw_sprite(tex, 0, s->output_width, s->output_height);
		gs_technique_end_pass(tech);
	}
	gs_technique_end(tech);
	gs_enable_framebuffer_srgb(previous_fb);
	gs_set_linear_srgb(previous_srgb);
}

static void virtual_source_render(void *data, gs_effect_t *)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	if (!s->effect || !s->p_image || !s->target || s->target_recursion_blocked ||
	    obs_source_removed(s->target))
		return;

	const uint32_t source_w = obs_source_get_width(s->target);
	const uint32_t source_h = obs_source_get_height(s->target);
	if (source_w == 0 || source_h == 0 || s->output_width == 0 ||
	    s->output_height == 0)
		return;

	const enum gs_color_space pref[] = {GS_CS_SRGB};
	const enum gs_color_space space =
		obs_source_get_color_space(s->target, 1, pref);
	if (!s->captured_this_frame) {
		if (!virtual_source_capture_target(s, source_w, source_h, space)) {
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
	virtual_source_draw_warp(s, tex, source_w, source_h);
	gs_blend_state_pop();
}

static void virtual_source_tick(void *data, float seconds)
{
	auto *s = static_cast<nyan_real_virtual_source *>(data);
	if (!s)
		return;
	s->captured_this_frame = false;

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
