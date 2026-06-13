// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
#include "remote_schema.h"

#include <obs-module.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>

#include "device_manager.h"
#include "device_registry.h"
#include "display-wall-source.h"
#include "virtual_source.h"

namespace {

// One mirrored dock row. label_key is the dock's locale key (single source
// for wording); read() fills the row's current value/options/enabled state;
// write() applies an incoming value; visible() hides model-specific rows.
// All lambdas are captureless so the table stays a plain static array.
struct remote_row {
	const char *key;
	const char *label_key;
	const char *type; // status | toggle | slider | combo | action | text | number
	double min = 0.0, max = 0.0, step = 0.0;
	void (*read)(obs_data_t *row) = nullptr;
	void (*write)(obs_data_t *msg) = nullptr;
	bool (*visible)() = nullptr;
};

struct remote_section {
	const char *label_key;
	const remote_row *rows;
	size_t count;
};

transport_traits current_traits()
{
	return traits_for(detected_transport_for(&g_device));
}

void set_v(obs_data_t *row, bool v)
{
	obs_data_set_bool(row, "v", v);
}
void set_v(obs_data_t *row, double v)
{
	obs_data_set_double(row, "v", v);
}
void set_v(obs_data_t *row, const char *v)
{
	obs_data_set_string(row, "v", v);
}
void set_enabled(obs_data_t *row, bool enabled)
{
	obs_data_set_bool(row, "enabled", enabled);
}

void push_option(obs_data_array_t *options, const std::string &value,
		 const char *label)
{
	obs_data_t *o = obs_data_create();
	obs_data_set_string(o, "v", value.c_str());
	obs_data_set_string(o, "label", label);
	obs_data_array_push_back(options, o);
	obs_data_release(o);
}

// --- ステータス ---------------------------------------------------------

const remote_row STATUS_ROWS[] = {
	{"st_hid", "dock.hid", "status", 0, 0, 0,
	 [](obs_data_t *r) {
		 const model_id m = detected_hid_model(&g_device);
		 set_v(r, m == MODEL_UNKNOWN
				  ? obs_module_text("detected_device.none")
				  : profile_for(m).name.c_str());
	 }},
	{"st_glasses", "dock.glasses_display", "status", 0, 0, 0,
	 [](obs_data_t *r) {
		 nyan_real_glasses_display_info g;
		 if (nyan_real_find_glasses_display(&g))
			 set_v(r, g.friendly_name.empty()
					  ? g.gdi_device.c_str()
					  : g.friendly_name.c_str());
		 else
			 set_v(r, obs_module_text("dock.glasses_display.none"));
	 }},
	{"st_transport", "dock.transport", "status", 0, 0, 0,
	 [](obs_data_t *r) {
		 set_v(r, obs_module_text(current_traits().name_key));
	 }},
	{"st_stream", "dock.stream", "status", 0, 0, 0,
	 [](obs_data_t *r) {
		 const bool enabled = g_device.connect_enabled.load(
			 std::memory_order_relaxed);
		 const bool connected =
			 g_device.connected.load(std::memory_order_relaxed);
		 set_v(r, obs_module_text(
				  !enabled ? "dock.stream.disabled"
					   : (connected
						      ? "dock.stream.connected"
						      : "dock.stream.waiting")));
	 }},
	{"st_pose", "dock.pose", "status", 0, 0, 0,
	 [](obs_data_t *r) {
		 const bool enabled = g_device.connect_enabled.load(
			 std::memory_order_relaxed);
		 const bool connected =
			 g_device.connected.load(std::memory_order_relaxed);
		 pose_snapshot p;
		 {
			 std::lock_guard<std::mutex> lk(g_device.state_mutex);
			 p = g_device.pose;
		 }
		 const char *key = "dock.pose.disconnected";
		 if (!enabled)
			 key = "dock.pose.disabled";
		 else if (connected && p.connected)
			 key = p.calibrated ? "dock.pose.calibrated"
					    : "dock.pose.calibrating";
		 set_v(r, obs_module_text(key));
	 }},
	{"st_sources", "dock.virtual_sources", "status", 0, 0, 0,
	 [](obs_data_t *r) {
		 set_v(r, std::to_string(g_device.virtual_source_count.load(
					 std::memory_order_relaxed))
				  .c_str());
	 }},
};

// --- デバイス操作 -------------------------------------------------------

const remote_row DEVICE_ROWS[] = {
	{"brightness", "brightness", "slider", 0, 20, 1,
	 [](obs_data_t *r) {
		 const int cur = g_device.brightness_current.load(
			 std::memory_order_relaxed);
		 const int autob = g_device.autobright_current.load(
			 std::memory_order_relaxed);
		 set_v(r, static_cast<double>(cur >= 0 && cur <= 20 ? cur : 0));
		 set_enabled(r, cur >= 0 && cur <= 20 && autob != 1);
	 },
	 [](obs_data_t *m) {
		 g_device.brightness_request.store(
			 static_cast<int>(
				 std::lround(obs_data_get_double(m, "v"))),
			 std::memory_order_relaxed);
	 },
	 []() { return current_traits().display_brightness; }},
	{"autobright", "autobright", "toggle", 0, 0, 0,
	 [](obs_data_t *r) {
		 const int autob = g_device.autobright_current.load(
			 std::memory_order_relaxed);
		 set_v(r, autob == 1);
		 set_enabled(r, autob >= 0);
	 },
	 [](obs_data_t *m) {
		 g_device.autobright_request.store(
			 obs_data_get_bool(m, "v") ? 1 : 0,
			 std::memory_order_relaxed);
	 },
	 []() { return current_traits().display_brightness; }},
	{"convergence_link", "convergence_link", "toggle", 0, 0, 0,
	 [](obs_data_t *r) {
		 set_v(r, g_device.convergence_link.load(
				  std::memory_order_relaxed));
		 set_enabled(r, g_device.display_distance_current.load(
				 std::memory_order_relaxed) != INT32_MIN);
	 },
	 [](obs_data_t *m) {
		 g_device.convergence_link.store(obs_data_get_bool(m, "v"),
						 std::memory_order_relaxed);
	 },
	 []() { return current_traits().display_brightness; }},
	{"displaymode", "displaymode", "combo", 0, 0, 0,
	 [](obs_data_t *r) {
		 const transport_traits tr = current_traits();
		 obs_data_array_t *options = obs_data_array_create();
		 for (size_t i = 0; i < tr.display_mode_count; i++)
			 push_option(options,
				     std::to_string(tr.display_modes[i].value),
				     obs_module_text(
					     tr.display_modes[i].label_key));
		 obs_data_set_array(r, "options", options);
		 obs_data_array_release(options);
		 const int cur = g_device.display_mode_current.load(
			 std::memory_order_relaxed);
		 set_v(r, std::to_string(cur).c_str());
		 set_enabled(r, cur >= 0);
	 },
	 [](obs_data_t *m) {
		 // Accept only values the detected family lists.
		 const int v = atoi(obs_data_get_string(m, "v"));
		 const transport_traits tr = current_traits();
		 for (size_t i = 0; i < tr.display_mode_count; i++) {
			 if (tr.display_modes[i].value == v) {
				 g_device.display_mode_request.store(
					 v, std::memory_order_relaxed);
				 return;
			 }
		 }
	 },
	 []() { return current_traits().display_mode_count > 0; }},
	{"st_eye", "dock.eye", "status", 0, 0, 0,
	 [](obs_data_t *r) {
		 const int present =
			 g_device.eye_present.load(std::memory_order_relaxed);
		 const int uvc =
			 g_device.eye_uvc.load(std::memory_order_relaxed);
		 const bool pending = g_device.eye_request.load(
					      std::memory_order_relaxed) >= 0;
		 const char *key;
		 if (pending)
			 key = "dock.eye.switching";
		 else if (present < 0)
			 key = "dock.eye.unknown";
		 else if (present == 0)
			 key = "dock.eye.absent";
		 else
			 key = uvc == 1 ? "dock.eye.uvc_on"
				        : "dock.eye.uvc_off";
		 set_v(r, obs_module_text(key));
	 },
	 nullptr, []() { return current_traits().eye_camera; }},
	{"eye_toggle", nullptr, "action", 0, 0, 0,
	 [](obs_data_t *r) {
		 const int present =
			 g_device.eye_present.load(std::memory_order_relaxed);
		 const int uvc =
			 g_device.eye_uvc.load(std::memory_order_relaxed);
		 const bool pending = g_device.eye_request.load(
					      std::memory_order_relaxed) >= 0;
		 obs_data_set_string(r, "label",
				     obs_module_text(uvc == 1
							     ? "dock.eye.disable"
							     : "dock.eye.enable"));
		 set_enabled(r, present == 1 && uvc >= 0 && !pending);
	 },
	 [](obs_data_t *) {
		 const int uvc =
			 g_device.eye_uvc.load(std::memory_order_relaxed);
		 g_device.eye_request.store(uvc == 1 ? 0 : 1,
					    std::memory_order_relaxed);
	 },
	 []() { return current_traits().eye_camera; }},
};

// --- メガネ画面・出力 ---------------------------------------------------

const remote_row OUTPUT_ROWS[] = {
	{"open_projector", "dock.open_projector", "action", 0, 0, 0,
	 [](obs_data_t *r) {
		 nyan_real_glasses_display_info g;
		 set_enabled(r, nyan_real_find_glasses_display(&g));
	 },
	 [](obs_data_t *) {
		 // Needs Qt; the dock poll consumes the request.
		 g_device.projector_request.store(true,
						  std::memory_order_relaxed);
	 }},
	{"auto_projector", "dock.auto_projector", "toggle", 0, 0, 0,
	 [](obs_data_t *r) {
		 set_v(r, g_device.auto_projector.load(
				  std::memory_order_relaxed));
	 },
	 [](obs_data_t *m) {
		 g_device.auto_projector.store(obs_data_get_bool(m, "v"),
					       std::memory_order_relaxed);
	 }},
	{"sbs_output", "sbs_output", "combo", 0, 0, 0,
	 [](obs_data_t *r) {
		 obs_data_array_t *options = obs_data_array_create();
		 push_option(options, "0", obs_module_text("sbs_output.auto"));
		 push_option(options, "1", obs_module_text("sbs_output.on"));
		 push_option(options, "2", obs_module_text("sbs_output.off"));
		 obs_data_set_array(r, "options", options);
		 obs_data_array_release(options);
		 set_v(r, std::to_string(g_device.sbs_output.load(
				 std::memory_order_relaxed))
				  .c_str());
	 },
	 [](obs_data_t *m) {
		 const int v = atoi(obs_data_get_string(m, "v"));
		 if (v >= 0 && v <= 2)
			 g_device.sbs_output.store(v,
						   std::memory_order_relaxed);
	 }},
	{"monitor_out", "dock.monitor_out", "combo", 0, 0, 0,
	 [](obs_data_t *r) {
		 // Same entries as the dock's combo: the two modes plus the
		 // present endpoints, plus the remembered-but-absent device.
		 int mode = g_device.monitor_out.load(std::memory_order_relaxed);
		 std::string sel_id, sel_name;
		 {
			 std::lock_guard<std::mutex> lk(g_device.settings_mutex);
			 sel_id = g_device.monitor_device_id;
			 sel_name = g_device.monitor_device_name;
		 }
		 obs_data_array_t *options = obs_data_array_create();
		 push_option(options, "@auto",
			     obs_module_text("dock.monitor_out.auto"));
		 push_option(options, "@keep",
			     obs_module_text("dock.monitor_out.keep"));
		 struct ctx_t {
			 obs_data_array_t *options;
			 const std::string *sel_id;
			 bool sel_found = false;
		 } ctx = {options, &sel_id, false};
		 obs_enum_audio_monitoring_devices(
			 [](void *data, const char *name, const char *id) {
				 auto *c = static_cast<ctx_t *>(data);
				 push_option(c->options, id ? id : "",
					     name ? name : "");
				 if (id && *c->sel_id == id)
					 c->sel_found = true;
				 return true;
			 },
			 &ctx);
		 std::string value = "@auto";
		 if (mode == MONITOR_OUT_KEEP) {
			 value = "@keep";
		 } else if (mode == MONITOR_OUT_DEVICE && !sel_id.empty()) {
			 value = sel_id;
			 if (!ctx.sel_found)
				 push_option(
					 options, sel_id,
					 ((sel_name.empty() ? sel_id : sel_name) +
					  obs_module_text(
						  "dock.monitor_out.missing_suffix"))
						 .c_str());
		 }
		 obs_data_set_array(r, "options", options);
		 obs_data_array_release(options);
		 set_v(r, value.c_str());
	 },
	 [](obs_data_t *m) {
		 const char *v = obs_data_get_string(m, "v");
		 if (strcmp(v, "@auto") == 0) {
			 g_device.monitor_out.store(MONITOR_OUT_AUTO_GLASSES,
						    std::memory_order_relaxed);
		 } else if (strcmp(v, "@keep") == 0) {
			 g_device.monitor_out.store(MONITOR_OUT_KEEP,
						    std::memory_order_relaxed);
		 } else if (*v) {
			 // Resolve the display name like the dock does; the
			 // page only sends the endpoint id.
			 struct ctx_t {
				 const char *id;
				 std::string name;
			 } ctx = {v, ""};
			 obs_enum_audio_monitoring_devices(
				 [](void *data, const char *name,
				    const char *id) {
					 auto *c = static_cast<ctx_t *>(data);
					 if (!id || strcmp(id, c->id) != 0)
						 return true;
					 c->name = name ? name : "";
					 return false;
				 },
				 &ctx);
			 {
				 std::lock_guard<std::mutex> lk(
					 g_device.settings_mutex);
				 g_device.monitor_device_id = v;
				 if (!ctx.name.empty())
					 g_device.monitor_device_name =
						 ctx.name;
			 }
			 g_device.monitor_out.store(MONITOR_OUT_DEVICE,
						    std::memory_order_relaxed);
		 }
		 // The per-connection latches live in the dock; ask its poll
		 // to re-apply the new choice.
		 g_device.monitor_rearm.store(true, std::memory_order_relaxed);
	 }},
	{"cursor_fence", "dock.cursor_fence", "toggle", 0, 0, 0,
	 [](obs_data_t *r) {
		 set_v(r, g_device.cursor_fence.load(
				  std::memory_order_relaxed));
	 },
	 [](obs_data_t *m) {
		 g_device.cursor_fence.store(obs_data_get_bool(m, "v"),
					     std::memory_order_relaxed);
	 }},
};

// --- 仮想スクリーン -----------------------------------------------------

const remote_row SCREEN_ROWS[] = {
	{"connect_enabled", "pose_follow", "toggle", 0, 0, 0,
	 [](obs_data_t *r) {
		 set_v(r, g_device.connect_enabled.load(
				  std::memory_order_relaxed));
	 },
	 [](obs_data_t *m) {
		 manager_set_connect_enabled(&g_device,
					     obs_data_get_bool(m, "v"));
	 }},
	{"prediction_ms", "prediction_ms", "slider", 0, 50, 1,
	 [](obs_data_t *r) {
		 set_v(r, static_cast<double>(g_device.prediction_ms.load(
			       std::memory_order_relaxed)));
	 },
	 [](obs_data_t *m) {
		 g_device.prediction_ms.store(
			 static_cast<float>(obs_data_get_double(m, "v")),
			 std::memory_order_relaxed);
	 }},
	{"fov_auto", "fov_auto", "toggle", 0, 0, 0,
	 [](obs_data_t *r) {
		 set_v(r, g_device.fov_auto.load(std::memory_order_relaxed));
	 },
	 [](obs_data_t *m) {
		 g_device.fov_auto.store(obs_data_get_bool(m, "v"),
					 std::memory_order_relaxed);
		 if (obs_data_get_bool(m, "v"))
			 manager_apply_model_settings(&g_device);
	 }},
	{"fov_deg", "fov_deg", "slider", 20, 100, 1,
	 [](obs_data_t *r) {
		 set_v(r, static_cast<double>(g_device.fov_deg.load(
			       std::memory_order_relaxed)));
		 set_enabled(r, !g_device.fov_auto.load(
					 std::memory_order_relaxed));
	 },
	 [](obs_data_t *m) {
		 g_device.fov_deg.store(
			 static_cast<float>(obs_data_get_double(m, "v")),
			 std::memory_order_relaxed);
	 }},
	{"screen_distance_m", "screen_distance_m", "slider",
	 MIN_SCREEN_DISTANCE_M, MAX_SCREEN_DISTANCE_M, 0.1,
	 [](obs_data_t *r) {
		 set_v(r, static_cast<double>(g_device.screen_distance_m.load(
			       std::memory_order_relaxed)));
	 },
	 [](obs_data_t *m) {
		 g_device.screen_distance_m.store(
			 static_cast<float>(obs_data_get_double(m, "v")),
			 std::memory_order_relaxed);
	 }},
	{"screen_size_factor", "screen_size_factor", "slider", 0.05, 4.0, 0.05,
	 [](obs_data_t *r) {
		 set_v(r, static_cast<double>(g_device.screen_size_factor.load(
			       std::memory_order_relaxed)));
	 },
	 [](obs_data_t *m) {
		 g_device.screen_size_factor.store(
			 static_cast<float>(obs_data_get_double(m, "v")),
			 std::memory_order_relaxed);
	 }},
	{"screen_curve", "screen_curve", "slider", 0.0, MAX_SCREEN_CURVE, 0.05,
	 [](obs_data_t *r) {
		 set_v(r, static_cast<double>(g_device.screen_curve.load(
			       std::memory_order_relaxed)));
	 },
	 [](obs_data_t *m) {
		 g_device.screen_curve.store(
			 static_cast<float>(obs_data_get_double(m, "v")),
			 std::memory_order_relaxed);
	 }},
	{"ipd_mm", "ipd_mm", "slider", MIN_IPD_MM, MAX_IPD_MM, 0.5,
	 [](obs_data_t *r) {
		 set_v(r, static_cast<double>(g_device.ipd_mm.load(
			       std::memory_order_relaxed)));
		 // Same actually-rendering-SBS gate as the dock's IPD row.
		 uint32_t w = g_glasses_display_width.load(
			 std::memory_order_relaxed);
		 uint32_t h = g_glasses_display_height.load(
			 std::memory_order_relaxed);
		 if (!w || !h) {
			 const model_profile &p =
				 profile_for(detected_hid_model(&g_device));
			 w = p.display_width;
			 h = p.display_height;
		 }
		 set_enabled(r, sbs_output_active(w, h));
	 },
	 [](obs_data_t *m) {
		 g_device.ipd_mm.store(
			 static_cast<float>(obs_data_get_double(m, "v")),
			 std::memory_order_relaxed);
	 }},
	{"st_screen_result", "dock.screen_result", "status", 0, 0, 0,
	 [](obs_data_t *r) {
		 const double fov = clampd(g_device.fov_deg.load(
						   std::memory_order_relaxed),
					   20.0, 100.0);
		 const double dist = clampd(
			 g_device.screen_distance_m.load(
				 std::memory_order_relaxed),
			 MIN_SCREEN_DISTANCE_M, MAX_SCREEN_DISTANCE_M);
		 const double size = clampd(g_device.screen_size_factor.load(
						    std::memory_order_relaxed),
					    0.05, 4.0);
		 const double diag_m = 2.0 * SCREEN_SIZE_UNIT_DISTANCE_M *
				       std::tan(fov * PI / 360.0) * size;
		 const double apparent =
			 2.0 * std::atan(diag_m / (2.0 * dist)) * 180.0 / PI;
		 char text[48];
		 snprintf(text, sizeof(text), "%.1f in / %.1f deg",
			  diag_m / 0.0254, apparent);
		 set_v(r, text);
	 }},
};

// --- 詳細設定 -----------------------------------------------------------

const remote_row ADVANCED_ROWS[] = {
	{"ip", "ip", "text", 0, 0, 0,
	 [](obs_data_t *r) {
		 std::lock_guard<std::mutex> lk(g_device.settings_mutex);
		 set_v(r, g_device.ip.c_str());
	 },
	 [](obs_data_t *m) {
		 int port;
		 {
			 std::lock_guard<std::mutex> lk(g_device.settings_mutex);
			 port = g_device.port;
		 }
		 manager_set_network(&g_device, obs_data_get_string(m, "v"),
				     port);
	 },
	 []() { return current_traits().uses_network_endpoint; }},
	{"port", "port", "number", 1, 65535, 1,
	 [](obs_data_t *r) {
		 std::lock_guard<std::mutex> lk(g_device.settings_mutex);
		 set_v(r, static_cast<double>(g_device.port));
	 },
	 [](obs_data_t *m) {
		 std::string ip;
		 {
			 std::lock_guard<std::mutex> lk(g_device.settings_mutex);
			 ip = g_device.ip;
		 }
		 manager_set_network(&g_device, ip,
				     static_cast<int>(
					     obs_data_get_double(m, "v")));
	 },
	 []() { return current_traits().uses_network_endpoint; }},
	{"mag_yaw", "mag_yaw", "toggle", 0, 0, 0,
	 [](obs_data_t *r) {
		 set_v(r, g_device.mag_yaw.load(std::memory_order_relaxed));
	 },
	 [](obs_data_t *m) {
		 manager_set_mag_yaw(&g_device, obs_data_get_bool(m, "v"));
	 }},
	{"debug_log", "debug_log", "toggle", 0, 0, 0,
	 [](obs_data_t *r) {
		 set_v(r, g_device.debug_log.load(std::memory_order_relaxed));
	 },
	 [](obs_data_t *m) {
		 g_device.debug_log.store(obs_data_get_bool(m, "v"),
					  std::memory_order_relaxed);
	 }},
	// 「設定を既定値に戻す」と「スマホリモコン」セクションは意図的に
	// 非対象: どちらもリモコン自身の接続を断ち切れる(リセットは
	// remote_enabled=false+トークンクリアを含む)ため、PC側からのみ。
};

const remote_section SECTIONS[] = {
	{"dock.status", STATUS_ROWS, std::size(STATUS_ROWS)},
	{"dock.device", DEVICE_ROWS, std::size(DEVICE_ROWS)},
	{"dock.output", OUTPUT_ROWS, std::size(OUTPUT_ROWS)},
	{"dock.screen", SCREEN_ROWS, std::size(SCREEN_ROWS)},
	{"dock.advanced", ADVANCED_ROWS, std::size(ADVANCED_ROWS)},
};

} // namespace

void remote_schema_build_cfg(obs_data_t *cfg)
{
	obs_data_array_t *sections = obs_data_array_create();
	for (const remote_section &sec : SECTIONS) {
		obs_data_array_t *rows = obs_data_array_create();
		for (size_t i = 0; i < sec.count; i++) {
			const remote_row &row = sec.rows[i];
			if (row.visible && !row.visible())
				continue;
			obs_data_t *r = obs_data_create();
			obs_data_set_string(r, "k", row.key);
			obs_data_set_string(r, "type", row.type);
			if (row.label_key)
				obs_data_set_string(
					r, "label",
					obs_module_text(row.label_key));
			if (strcmp(row.type, "slider") == 0 ||
			    strcmp(row.type, "number") == 0) {
				obs_data_set_double(r, "min", row.min);
				obs_data_set_double(r, "max", row.max);
				obs_data_set_double(r, "step", row.step);
			}
			obs_data_set_bool(r, "enabled", true);
			if (row.read)
				row.read(r);
			obs_data_array_push_back(rows, r);
			obs_data_release(r);
		}
		if (obs_data_array_count(rows)) {
			obs_data_t *s = obs_data_create();
			obs_data_set_string(s, "label",
					    obs_module_text(sec.label_key));
			obs_data_set_array(s, "rows", rows);
			obs_data_array_push_back(sections, s);
			obs_data_release(s);
		}
		obs_data_array_release(rows);
	}
	obs_data_set_array(cfg, "sections", sections);
	obs_data_array_release(sections);
}

bool remote_schema_apply(obs_data_t *msg)
{
	const char *key = obs_data_get_string(msg, "k");
	if (!key || !*key)
		return false;
	for (const remote_section &sec : SECTIONS) {
		for (size_t i = 0; i < sec.count; i++) {
			const remote_row &row = sec.rows[i];
			if (strcmp(row.key, key) != 0)
				continue;
			if (!row.write || (row.visible && !row.visible()))
				return false;
			if (strcmp(row.type, "slider") == 0 ||
			    strcmp(row.type, "number") == 0)
				obs_data_set_double(
					msg, "v",
					clampd(obs_data_get_double(msg, "v"),
					       row.min, row.max));
			row.write(msg);
			return true;
		}
	}
	return false;
}
