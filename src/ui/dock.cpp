// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "dock.h"

#include <windows.h>

#ifdef NYAN_REAL_3DOF_WITH_QT_DOCK
#include <QAbstractItemView>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QDockWidget>
#include <QFormLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QPixmap>
#include <QPointer>
#include <QPushButton>
#include <QScreen>
#include <QSignalBlocker>
#include <QSlider>
#include <QScrollArea>
#include <QSize>
#include <QSizePolicy>
#include <QFrame>
#include <QMouseEvent>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QWheelEvent>
#endif

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>

#include "cursor_fence.h"
#include "device_manager.h"
#include "device_registry.h"
#include "endpoint_volume.h" // output-device volume/mute (shared Win32)
#include "monitor_enum.h"
#include "nyan_host.h"
#include "nyan_log.h"
#include "nyan_section.h" // shared collapsible DockSection (also used standalone)
#include "nyan_toggle.h"  // modern ToggleSwitch + add_switch_row (also standalone)
#include "qrcodegen.hpp"
#include "remote_control.h"
#include "tooltip_util.h"

#ifdef NYAN_REAL_3DOF_WITH_QT_DOCK
// Word-wrapping tooltip from a locale key (see tooltip_util.h).
static QString tip(const char *locale_key)
{
	return QString::fromStdString(wrapped_tooltip(locale_key));
}

// OBS resolves the fullscreen-projector monitor argument as an index into
// Qt's screen list. What QScreen::name() returns on Windows changed over
// time: before Qt 6.4 it was the GDI device name ("\\.\DISPLAY2"), from 6.4
// on it is the DISPLAYCONFIG friendly monitor name ("Air 2"). Try both, then
// fall back to the native geometry for duplicate or missing names.
static int projector_monitor_index(const nyan_real_glasses_display_info &display)
{
	const QList<QScreen *> screens = QGuiApplication::screens();

	const QString gdi = QString::fromStdString(display.gdi_device);
	for (int i = 0; i < screens.size(); ++i) {
		if (screens[i]->name() == gdi)
			return i;
	}

	const QString friendly = QString::fromStdString(display.friendly_name);
	int name_index = -1;
	int name_matches = 0;
	if (!friendly.isEmpty()) {
		for (int i = 0; i < screens.size(); ++i) {
			if (screens[i]->name() == friendly) {
				name_index = i;
				name_matches++;
			}
		}
	}
	if (name_matches == 1)
		return name_index;

	if (display.has_rect) {
		int size_index = -1;
		int size_matches = 0;
		for (int i = 0; i < screens.size(); ++i) {
			const QScreen *screen = screens[i];
			const qreal dpr = screen->devicePixelRatio();
			const QRect geo = screen->geometry();
			if (qRound(geo.width() * dpr) !=
				    static_cast<int>(display.width) ||
			    qRound(geo.height() * dpr) !=
				    static_cast<int>(display.height))
				continue;
			// Positions are exact at 100 % scaling; with mixed
			// per-monitor DPI Qt remaps origins, so accept a
			// small drift before falling back to a size-only
			// unique match.
			if (std::abs(qRound(geo.x() * dpr) - display.x) <= 2 &&
			    std::abs(qRound(geo.y() * dpr) - display.y) <= 2)
				return i;
			size_index = i;
			size_matches++;
		}
		if (size_matches == 1)
			return size_index;
	}

	return name_matches > 0 ? name_index : -1;
}

// Opens the virtual screen's fullscreen output on the glasses display via the
// host (OBS: source projector; standalone: own fullscreen window). UI thread.
static bool open_glasses_source_projector(bool log_failure)
{
	nyan_real_glasses_display_info display;
	if (!nyan_real_find_glasses_display(&display)) {
		if (log_failure)
			nyan_log(NYAN_LOG_WARNING,
			     "[obs-nyan-real-3dof] no glasses display present (EDID match)");
		return false;
	}
	const int monitor = projector_monitor_index(display);
	if (monitor < 0) {
		if (log_failure) {
			std::string screen_names;
			for (QScreen *screen : QGuiApplication::screens()) {
				screen_names += '\'';
				screen_names += screen->name().toStdString();
				screen_names += "' ";
			}
			nyan_log(NYAN_LOG_WARNING,
			     "[obs-nyan-real-3dof] glasses display %s ('%s') not matched; Qt screens: %s",
			     display.gdi_device.c_str(),
			     display.friendly_name.c_str(),
			     screen_names.c_str());
		}
		return false;
	}
	return nyan_open_glasses_output(monitor, log_failure);
}

// Brand tokens that appear in the glasses' USB audio endpoint names
// ("スピーカー (XREAL Air 2 Pro)", "nreal light Audio", ...). EPSON is left
// out on purpose: projector audio endpoints share it.
static bool is_glasses_audio_name(const char *name)
{
	static const char *brands[] = {"xreal", "nreal",  "viture",
				       "rokid", "rayneo", "moverio"};
	if (!name)
		return false;
	std::string n = name;
	std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) {
		return static_cast<char>(std::tolower(c));
	});
	for (const char *b : brands) {
		if (n.find(b) != std::string::npos)
			return true;
	}
	return false;
}

// Monitoring-device enumeration result for the dock's output selector.
struct monitoring_device_match {
	std::string name;
	std::string id;
	bool found = false;
};

// Find the glasses' USB audio endpoint by brand token.
static monitoring_device_match find_glasses_monitoring_device()
{
	monitoring_device_match m;
	nyan_enum_audio_outputs(
		[](void *data, const char *name, const char *id) {
			auto *m = static_cast<monitoring_device_match *>(data);
			if (!is_glasses_audio_name(name))
				return true;
			m->name = name ? name : "";
			m->id = id ? id : "";
			m->found = true;
			return false;
		},
		&m);
	return m;
}

// Find a monitoring device by its WASAPI endpoint id - the persisted identity
// of the dock's device choice. Ids are stable across reconnects, while names
// can change with the connection state.
static monitoring_device_match
find_monitoring_device_by_id(const std::string &want_id)
{
	struct ctx_t {
		const std::string *want;
		monitoring_device_match m;
	} c = {&want_id, {}};
	if (want_id.empty())
		return c.m;
	nyan_enum_audio_outputs(
		[](void *data, const char *name, const char *id) {
			auto *c = static_cast<ctx_t *>(data);
			if (!id || *c->want != id)
				return true;
			c->m.name = name ? name : "";
			c->m.id = id;
			c->m.found = true;
			return false;
		},
		&c);
	return c.m;
}

// Point OBS's audio monitoring at the device. Returns true once monitoring
// points at it (already or newly set); the caller latches the success and
// re-arms when the endpoint disappears.
static bool apply_monitoring_device(const monitoring_device_match &m,
				    const char *why)
{
	std::string cur_name;
	std::string cur_id;
	nyan_audio_monitor_get(cur_name, cur_id);
	if (!cur_id.empty() && m.id == cur_id) {
		// Already the configured device - but monitors created while
		// the endpoint was still enumerating (OBS launch racing the
		// glasses' USB audio, Bluetooth reconnecting) failed with
		// AUDCLNT_E_DEVICE_INVALIDATED, and OBS never retries them on
		// its own. The endpoint provably exists right now (it was
		// just enumerated), so rebuild all monitors against it. Runs
		// once per appearance via the caller's latch.
		nyan_audio_monitor_reset();
		nyan_log(NYAN_LOG_INFO,
		     "[obs-nyan-real-3dof] audio monitoring re-initialized ('%s' is ready)",
		     m.name.c_str());
		return true;
	}
	if (!nyan_audio_monitor_set(m.name, m.id))
		return false;
	nyan_log(NYAN_LOG_INFO,
	     "[obs-nyan-real-3dof] audio monitoring device -> '%s' (%s)",
	     m.name.c_str(), why);
	return true;
}

class NoWheelSpinBox final : public QSpinBox {
public:
	using QSpinBox::QSpinBox;

protected:
	void wheelEvent(QWheelEvent *event) override { event->ignore(); }
};

class NoWheelDoubleSpinBox final : public QDoubleSpinBox {
public:
	using QDoubleSpinBox::QDoubleSpinBox;

protected:
	void wheelEvent(QWheelEvent *event) override { event->ignore(); }
};

// The dock lives in a scroll area; a stray wheel over the display-mode combo
// would send a mode-switch command to the glasses.
class NoWheelComboBox final : public QComboBox {
public:
	using QComboBox::QComboBox;

protected:
	void wheelEvent(QWheelEvent *event) override { event->ignore(); }
};

// Same reason for the value sliders: scrolling the dock should not nudge FOV /
// distance / IPD etc. The wheel still scrolls the dock (event is ignored, so it
// bubbles to the scroll area).
class NoWheelSlider final : public QSlider {
public:
	using QSlider::QSlider;

protected:
	void wheelEvent(QWheelEvent *event) override { event->ignore(); }
};

// Focus-mode picker: "none (whole wall)" + each non-glasses display. The wall
// displays change at runtime, so the list is rebuilt right before the popup
// opens. Item data is the Windows number (0 = off); the value is read from /
// written to g_device.focus_display.
class FocusComboBox final : public QComboBox {
public:
	using QComboBox::QComboBox;
	void showPopup() override
	{
		repopulate();
		QComboBox::showPopup();
	}
	void repopulate()
	{
		const int cur =
			g_device.focus_display.load(std::memory_order_relaxed);
		QSignalBlocker block(this);
		clear();
		addItem(nyan_text("dock.focus_off"), 0);
		int sel = 0;
		for (const monitor_entry &m : filter_monitors(
			     enumerate_monitors(), /*include_primary=*/true,
			     /*exclude_glasses=*/true, "", "")) {
			addItem(QString::fromStdString(m.label),
				m.windows_number);
			if (m.windows_number == cur)
				sel = count() - 1;
		}
		setCurrentIndex(sel);
	}

protected:
	void wheelEvent(QWheelEvent *event) override { event->ignore(); }
};

// QLabel with a left-click callback (the remote's QR code rotates its token
// on click). Same no-moc pattern as DockSectionHeader.
class ClickableLabel final : public QLabel {
public:
	using QLabel::QLabel;
	std::function<void()> on_click;

protected:
	void mousePressEvent(QMouseEvent *event) override
	{
		if (event->button() == Qt::LeftButton && on_click)
			on_click();
		QLabel::mousePressEvent(event);
	}
};


class NyanRealDock final : public QScrollArea {
public:
	explicit NyanRealDock(QWidget *parent = nullptr) : QScrollArea(parent)
	{
		setWidgetResizable(true);
		setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
		setMinimumSize(240, 180);
		setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Ignored);

		auto *content = new QWidget(this);
		content->setMinimumWidth(220);
		auto *root = new QVBoxLayout(content);
		root->setContentsMargins(10, 10, 10, 10);
		root->setSpacing(8);

		// Top action row: just Center. The pose-follow (3DoF) toggle and
		// Recalibrate moved into the virtual-screen section below - the eye
		// icon here was unlabelled and unclear.
		auto *action_row_1 = new QHBoxLayout();
		recenter = new QPushButton(nyan_text("recenter"), content);
		recenter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
		action_row_1->addWidget(recenter);
		root->addLayout(action_row_1);


		auto *status_body = new QWidget(content);
		auto *status_form = new QFormLayout(status_body);
		status_form->setContentsMargins(16, 0, 0, 4);
		hid_label = new QLabel(status_body);
		glasses_display_label = new QLabel(status_body);
		transport_label = new QLabel(status_body);
		stream_label = new QLabel(status_body);
		pose_label = new QLabel(status_body);
		virtual_label = new QLabel(status_body);
		status_form->addRow(nyan_text("dock.hid"), hid_label);
		status_form->addRow(nyan_text("dock.glasses_display"),
				    glasses_display_label);
		status_form->addRow(nyan_text("dock.transport"), transport_label);
		status_form->addRow(nyan_text("dock.stream"), stream_label);
		status_form->addRow(nyan_text("dock.pose"), pose_label);
		// "Virtual sources" counts OBS sources; the standalone has none, so
		// the host hides this row (nyan_caps). The label still exists for the
		// refresh path; it just stays out of the layout.
		if (nyan_caps().virtual_sources)
			status_form->addRow(nyan_text("dock.virtual_sources"),
					    virtual_label);
		else
			virtual_label->setVisible(false);
		status_section = new DockSection(nyan_text("dock.status"),
						 status_body, content);
		root->addWidget(status_section);

		// Model-specific hardware controls; the rows follow the
		// detected device and the whole section hides when none apply.
		auto *device_body = new QWidget(content);
		device_form = new QFormLayout(device_body);
		device_form->setContentsMargins(16, 0, 0, 4);
		brightness_spin = new NoWheelDoubleSpinBox(device_body);
		brightness_spin->setRange(0.0, 20.0);
		brightness_spin->setDecimals(0);
		brightness_spin->setSingleStep(1.0);
		brightness_row = make_double_slider(device_body, brightness_spin,
						    &brightness_slider,
						    BRIGHTNESS_SLIDER_SCALE);
		brightness_row->setToolTip(
			tip("brightness_tooltip"));
		device_form->addRow(nyan_text("brightness"),
				    brightness_row);
		autobright_box = new ToggleSwitch(device_body);
		autobright_box->setToolTip(
			tip("autobright_tooltip"));
		autobright_row = add_switch_row(device_form, nyan_text("autobright"),
						autobright_box);
		convergence_box = new ToggleSwitch(device_body);
		convergence_box->setToolTip(
			tip("convergence_link_tooltip"));
		convergence_row = add_switch_row(
			device_form, nyan_text("convergence_link"), convergence_box);
		display_mode_combo = new NoWheelComboBox(device_body);
		display_mode_combo->setToolTip(
			tip("displaymode_tooltip"));
		device_form->addRow(nyan_text("displaymode"),
				    display_mode_combo);
		eye_label = new QLabel(device_body);
		device_form->addRow(nyan_text("dock.eye"), eye_label);
		eye_button = new QPushButton(device_body);
		eye_button->setToolTip(tip("dock.eye_tooltip"));
		device_form->addRow(eye_button);
		device_section = new DockSection(nyan_text("dock.device"),
						 device_body, content);
		root->addWidget(device_section);

		auto *output_body = new QWidget(content);
		auto *output_form = new QFormLayout(output_body);
		output_form->setContentsMargins(16, 0, 0, 4);
		projector_button = new QPushButton(
			nyan_text("dock.open_projector"), output_body);
		projector_button->setToolTip(
			tip("dock.open_projector_tooltip"));
		auto_projector_box = new ToggleSwitch(output_body);
		auto_projector_box->setToolTip(
			tip("dock.auto_projector_tooltip"));
		// The glasses projector is an OBS notion; the standalone opens its own
		// fullscreen window automatically, so the host hides both controls
		// (nyan_caps). They stay alive for the refresh/connect paths.
		if (nyan_caps().projector) {
			output_form->addRow(projector_button);
			add_switch_row(output_form, nyan_text("dock.auto_projector"),
				       auto_projector_box);
		} else {
			projector_button->setVisible(false);
			auto_projector_box->setVisible(false);
		}
		sbs_combo = new NoWheelComboBox(output_body);
		sbs_combo->addItem(nyan_text("sbs_output.auto"), 0);
		sbs_combo->addItem(nyan_text("sbs_output.on"), 1);
		sbs_combo->addItem(nyan_text("sbs_output.off"), 2);
		sbs_combo->setToolTip(tip("sbs_output_tooltip"));
		output_form->addRow(nyan_text("sbs_output"), sbs_combo);
		monitor_combo = new NoWheelComboBox(output_body);
		monitor_combo->setToolTip(
			tip("dock.monitor_out_tooltip"));
		output_form->addRow(nyan_text("dock.monitor_out"),
				    monitor_combo);
		// Volume + mute of the chosen output device (its WASAPI endpoint
		// master). The glasses are usually not the Windows default, so the
		// taskbar slider cannot reach them; this controls them directly.
		volume_row = new QWidget(output_body);
		auto *volume_layout = new QHBoxLayout(volume_row);
		volume_layout->setContentsMargins(0, 0, 0, 0);
		volume_layout->setSpacing(6);
		volume_slider = new NoWheelSlider(Qt::Horizontal, volume_row);
		volume_slider->setRange(0, 100);
		mute_box = new ToggleSwitch(nyan_text("dock.mute"), volume_row);
		volume_layout->addWidget(volume_slider, 1);
		volume_layout->addWidget(mute_box);
		output_form->addRow(nyan_text("dock.volume"), volume_row);
		cursor_fence_box = new ToggleSwitch(output_body);
		cursor_fence_box->setToolTip(
			tip("dock.cursor_fence_tooltip"));
		add_switch_row(output_form, nyan_text("dock.cursor_fence"),
			       cursor_fence_box);
		output_section = new DockSection(nyan_text("dock.output"),
						 output_body, content);
		root->addWidget(output_section);

		auto *screen_body = new QWidget(content);
		auto *screen_form = new QFormLayout(screen_body);
		screen_form->setContentsMargins(16, 0, 0, 4);
		prediction_spin = new NoWheelDoubleSpinBox(screen_body);
		prediction_spin->setRange(0.0, 50.0);
		prediction_spin->setDecimals(0);
		prediction_spin->setSingleStep(1.0);
		fov_auto_box = new ToggleSwitch(screen_body);
		fov_spin = new NoWheelDoubleSpinBox(screen_body);
		fov_spin->setRange(20.0, 100.0);
		fov_spin->setDecimals(0);
		fov_spin->setSingleStep(1.0);
		distance_spin = new NoWheelDoubleSpinBox(screen_body);
		distance_spin->setRange(MIN_SCREEN_DISTANCE_M,
					MAX_SCREEN_DISTANCE_M);
		distance_spin->setDecimals(1);
		distance_spin->setSingleStep(0.1);
		size_spin = new NoWheelDoubleSpinBox(screen_body);
		size_spin->setRange(0.05, 4.0);
		size_spin->setDecimals(2);
		size_spin->setSingleStep(0.05);
		curve_spin = new NoWheelDoubleSpinBox(screen_body);
		curve_spin->setRange(0.0, MAX_SCREEN_CURVE);
		curve_spin->setDecimals(2);
		curve_spin->setSingleStep(0.05);
		ipd_spin = new NoWheelDoubleSpinBox(screen_body);
		ipd_spin->setRange(MIN_IPD_MM, MAX_IPD_MM);
		ipd_spin->setDecimals(1);
		ipd_spin->setSingleStep(0.5);
		screen_label = new QLabel(screen_body);
		// Build the rows, then add them in everyday-first order (rarely-changed
		// FOV / prediction / IPD sink to the bottom).
		auto *prediction_row =
			make_double_slider(screen_body, prediction_spin,
					   &prediction_slider, PREDICTION_SLIDER_SCALE);
		auto *fov_row = make_double_slider(screen_body, fov_spin, &fov_slider,
						   FOV_SLIDER_SCALE);
		distance_row = make_double_slider(screen_body, distance_spin,
						  &distance_slider,
						  DISTANCE_SLIDER_SCALE);
		// The base text; refresh() appends the detected model's optical focal
		// distance as SBS comfort guidance.
		distance_row->setToolTip(tip("screen_distance_tooltip"));
		auto *size_row = make_double_slider(screen_body, size_spin, &size_slider,
						    SIZE_SLIDER_SCALE);
		auto *curve_row = make_double_slider(screen_body, curve_spin,
						     &curve_slider, CURVE_SLIDER_SCALE);
		auto *ipd_row = make_double_slider(screen_body, ipd_spin, &ipd_slider,
						   IPD_SLIDER_SCALE);
		ipd_row->setToolTip(tip("ipd_tooltip"));
		focus_combo = new FocusComboBox(screen_body);
		focus_combo->setToolTip(tip("dock.focus_tooltip"));
		focus_combo->repopulate();
		// Pose-follow (3DoF) toggle: was an unlabelled eye icon in the top
		// action row; now a labelled switch right under the focus picker.
		connect_box = new ToggleSwitch(screen_body);
		connect_box->setToolTip(tip("dock.pose_follow_tooltip"));
		offscreen_box = new ToggleSwitch(screen_body);
		offscreen_box->setToolTip(tip("dock.offscreen_indicator_tooltip"));
		recalibrate =
			new QPushButton(nyan_text("recalibrate"), screen_body);

		screen_form->addRow(nyan_text("screen_distance_m"), distance_row);
		screen_form->addRow(nyan_text("screen_size_factor"), size_row);
		screen_form->addRow(nyan_text("screen_curve"), curve_row);
		screen_form->addRow(nyan_text("dock.focus"), focus_combo);
		add_switch_row(screen_form, nyan_text("pose_follow"), connect_box);
		add_switch_row(screen_form, nyan_text("dock.offscreen_indicator"),
			       offscreen_box);
		add_switch_row(screen_form, nyan_text("fov_auto"), fov_auto_box);
		screen_form->addRow(nyan_text("fov_deg"), fov_row);
		screen_form->addRow(nyan_text("prediction_ms"), prediction_row);
		screen_form->addRow(nyan_text("ipd_mm"), ipd_row);
		screen_form->addRow(nyan_text("dock.screen_result"), screen_label);
		// Gyro-bias recalibration: rarely used, at the very bottom.
		screen_form->addRow(recalibrate);
		screen_section = new DockSection(nyan_text("dock.screen"),
						 screen_body, content);
		root->addWidget(screen_section);

		// Phone remote: a LAN touchpad page served by the plugin.
		// Collapsed and disabled by default; enabling shows the QR
		// code that opens the page (URL carries the access token).
		auto *remote_body = new QWidget(content);
		auto *remote_form = new QFormLayout(remote_body);
		remote_form->setContentsMargins(16, 0, 0, 4);
		remote_enable_box = new ToggleSwitch(remote_body);
		remote_enable_box->setToolTip(
			tip("dock.remote_enable_tooltip"));
		add_switch_row(remote_form, nyan_text("dock.remote_enable"),
			       remote_enable_box);
		remote_port_spin = new NoWheelSpinBox(remote_body);
		remote_port_spin->setRange(1024, 65535);
		remote_form->addRow(nyan_text("dock.remote_port"),
				    remote_port_spin);
		remote_qr_label = new ClickableLabel(remote_body);
		remote_qr_label->setAlignment(Qt::AlignCenter);
		remote_qr_label->setVisible(false);
		remote_qr_label->setCursor(Qt::PointingHandCursor);
		remote_qr_label->setToolTip(tip("dock.remote_qr_tooltip"));
		remote_form->addRow(remote_qr_label);
		remote_url_label = new QLabel(remote_body);
		remote_url_label->setTextInteractionFlags(
			Qt::TextSelectableByMouse);
		remote_url_label->setAlignment(Qt::AlignCenter);
		remote_url_label->setWordWrap(true);
		remote_url_label->setVisible(false);
		remote_form->addRow(remote_url_label);
		remote_kick_button = new QPushButton(
			nyan_text("dock.remote_kick"), remote_body);
		remote_kick_button->setToolTip(
			tip("dock.remote_kick_tooltip"));
		remote_kick_button->setVisible(false);
		remote_form->addRow(remote_kick_button);
		remote_section = new DockSection(
			nyan_text("dock.remote"), remote_body, content);
		root->addWidget(remote_section);

		// Rarely-touched settings, collapsed by default. The One-family
		// TCP endpoint rows still follow the detected device.
		auto *advanced_body = new QWidget(content);
		advanced_form = new QFormLayout(advanced_body);
		advanced_form->setContentsMargins(16, 0, 0, 4);
		ip_edit = new QLineEdit(advanced_body);
		port_spin = new NoWheelSpinBox(advanced_body);
		port_spin->setRange(1, 65535);
		advanced_form->addRow(nyan_text("ip"), ip_edit);
		advanced_form->addRow(nyan_text("port"), port_spin);
		mag_yaw_box = new ToggleSwitch(advanced_body);
		debug_box = new ToggleSwitch(advanced_body);
		// Resets every dock setting; lives at the bottom of the
		// advanced section, away from the everyday tracker buttons.
		auto *reset_defaults = new QPushButton(
			nyan_text("reset_defaults"), advanced_body);
		add_switch_row(advanced_form, nyan_text("mag_yaw"), mag_yaw_box);
		add_switch_row(advanced_form, nyan_text("debug_log"), debug_box);
		advanced_form->addRow(reset_defaults);
		advanced_section = new DockSection(
			nyan_text("dock.advanced"), advanced_body, content);
		root->addWidget(advanced_section);
		root->addStretch();
		setWidget(content);

		// Collapse-state persistence: header clicks write the bit
		// mask; refresh() applies external changes (settings load).
		const auto bind_section = [this](DockSection *s, uint32_t bit) {
			s->on_toggled = [this, bit](bool open) {
				uint32_t v = g_device.dock_collapsed.load(
					std::memory_order_relaxed);
				v = open ? (v & ~bit) : (v | bit);
				g_device.dock_collapsed.store(
					v, std::memory_order_relaxed);
				last_collapsed_seen = v;
			};
		};
		bind_section(status_section, DOCK_SECTION_STATUS);
		bind_section(device_section, DOCK_SECTION_DEVICE);
		bind_section(output_section, DOCK_SECTION_OUTPUT);
		bind_section(screen_section, DOCK_SECTION_SCREEN);
		bind_section(remote_section, DOCK_SECTION_REMOTE);
		bind_section(advanced_section, DOCK_SECTION_ADVANCED);

		// The eye-icon toggles head-pose follow (3DoF). It no longer closes
		// the IMU connection - the device stays connected and tracking, the
		// view just freezes to a head-locked mirror while off.
		QObject::connect(connect_box, &QAbstractButton::toggled, this,
				 [](bool checked) {
					 g_device.pose_follow.store(
						 checked,
						 std::memory_order_relaxed);
				 });
		QObject::connect(brightness_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.brightness_request.store(
						 static_cast<int>(
							 std::lround(value)),
						 std::memory_order_relaxed);
				 });
		QObject::connect(autobright_box, &QAbstractButton::toggled, this,
				 [](bool checked) {
					 g_device.autobright_request.store(
						 checked ? 1 : 0,
						 std::memory_order_relaxed);
				 });
		QObject::connect(convergence_box, &QAbstractButton::toggled, this,
				 [](bool checked) {
					 g_device.convergence_link.store(
						 checked,
						 std::memory_order_relaxed);
				 });
		QObject::connect(offscreen_box, &QAbstractButton::toggled, this,
				 [](bool checked) {
					 g_device.offscreen_indicator.store(
						 checked,
						 std::memory_order_relaxed);
				 });
		QObject::connect(focus_combo,
				 QOverload<int>::of(&QComboBox::activated), this,
				 [this](int index) {
					 manager_set_focus_display(
						 &g_device,
						 focus_combo->itemData(index)
							 .toInt());
				 });
		// activated fires only on user interaction, so the periodic
		// refresh sync below cannot echo a request back to the device.
		QObject::connect(display_mode_combo,
				 QOverload<int>::of(&QComboBox::activated), this,
				 [this](int index) {
					 const QVariant v =
						 display_mode_combo->itemData(index);
					 if (v.isValid())
						 g_device.display_mode_request.store(
							 v.toInt(),
							 std::memory_order_relaxed);
				 });
		QObject::connect(sbs_combo,
				 QOverload<int>::of(&QComboBox::activated), this,
				 [this](int index) {
					 g_device.sbs_output.store(
						 sbs_combo->itemData(index).toInt(),
						 std::memory_order_relaxed);
				 });
		QObject::connect(eye_button, &QPushButton::clicked, this,
				 [this]() {
					 const int uvc = g_device.eye_uvc.load(
						 std::memory_order_relaxed);
					 g_device.eye_request.store(
						 uvc == 1 ? 0 : 1,
						 std::memory_order_relaxed);
					 refresh();
				 });
		QObject::connect(ip_edit, &QLineEdit::editingFinished, this, [this]() {
			manager_set_network(&g_device, ip_edit->text().trimmed().toStdString(),
					    port_spin->value());
		});
		QObject::connect(port_spin, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
				 this, [this](int value) {
					 manager_set_network(
						 &g_device,
						 ip_edit->text().trimmed().toStdString(), value);
				 });
		QObject::connect(recenter, &QPushButton::clicked, this,
				 []() { manager_recenter(&g_device); });
		QObject::connect(recalibrate, &QPushButton::clicked, this,
				 []() { manager_recalibrate(&g_device); });
		QObject::connect(projector_button, &QPushButton::clicked, this,
				 [this]() {
					 if (open_glasses_source_projector(true))
						 auto_projector_opened = true;
				 });
		QObject::connect(auto_projector_box, &QAbstractButton::toggled, this,
				 [](bool checked) {
					 g_device.auto_projector.store(
						 checked,
						 std::memory_order_relaxed);
				 });
		// activated fires only on user picks, not on the poll's
		// programmatic rebuilds.
		QObject::connect(
			monitor_combo, QOverload<int>::of(&QComboBox::activated),
			this, [this](int index) {
				const QString id =
					monitor_combo->itemData(index).toString();
				if (id == QStringLiteral("@auto")) {
					g_device.monitor_out.store(
						MONITOR_OUT_AUTO_GLASSES,
						std::memory_order_relaxed);
				} else if (id == QStringLiteral("@keep")) {
					g_device.monitor_out.store(
						MONITOR_OUT_KEEP,
						std::memory_order_relaxed);
				} else {
					// Raw endpoint name (no state suffix)
					// kept alongside the label.
					const QString name =
						monitor_combo
							->itemData(index,
								   Qt::UserRole + 1)
							.toString();
					{
						std::lock_guard<std::mutex> lk(
							g_device.settings_mutex);
						g_device.monitor_device_id =
							id.toStdString();
						g_device.monitor_device_name =
							name.toStdString();
					}
					g_device.monitor_out.store(
						MONITOR_OUT_DEVICE,
						std::memory_order_relaxed);
				}
				// Re-arm so the choice applies on the next
				// poll without replugging.
				auto_monitor_applied = false;
				monitor_device_applied = false;
				refresh();
			});
		// Volume/mute act on the live output endpoint. valueChanged/toggled
		// only fire from user input here - refresh_volume() blocks signals
		// when it writes the actual device state back into the widgets.
		QObject::connect(volume_slider, &QSlider::valueChanged, this,
				 [this](int v) {
					 const std::string id = current_output_id();
					 if (!id.empty())
						 endpoint_volume_set(id,
								     v / 100.0f);
				 });
		QObject::connect(mute_box, &QAbstractButton::toggled, this,
				 [this](bool on) {
					 const std::string id = current_output_id();
					 if (!id.empty())
						 endpoint_volume_set_mute(id, on);
				 });
		// The fence itself rises/falls on the next poll tick, which
		// also knows the current glasses-display rect.
		QObject::connect(cursor_fence_box, &QAbstractButton::toggled, this,
				 [this](bool checked) {
					 g_device.cursor_fence.store(
						 checked,
						 std::memory_order_relaxed);
					 refresh();
				 });
		QObject::connect(reset_defaults, &QPushButton::clicked, this, [this]() {
			manager_reset_defaults(&g_device);
			refresh();
		});
		QObject::connect(prediction_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.prediction_ms.store(static_cast<float>(value),
								      std::memory_order_relaxed);
				 });
		QObject::connect(fov_auto_box, &QAbstractButton::toggled, this, [this](bool checked) {
			g_device.fov_auto.store(checked, std::memory_order_relaxed);
			set_double_enabled(fov_spin, fov_slider, !checked);
			if (checked)
				manager_apply_model_settings(&g_device);
		});
		QObject::connect(fov_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.fov_deg.store(static_cast<float>(value),
								std::memory_order_relaxed);
				 });
		QObject::connect(distance_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.screen_distance_m.store(
						 static_cast<float>(value),
						 std::memory_order_relaxed);
				 });
		QObject::connect(size_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.screen_size_factor.store(
						 static_cast<float>(value),
						 std::memory_order_relaxed);
				 });
		QObject::connect(curve_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.screen_curve.store(static_cast<float>(value),
								     std::memory_order_relaxed);
				 });
		QObject::connect(ipd_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.ipd_mm.store(static_cast<float>(value),
							       std::memory_order_relaxed);
				 });
		QObject::connect(remote_enable_box, &QAbstractButton::toggled, this,
				 [this](bool checked) {
					 g_device.remote_enabled.store(
						 checked,
						 std::memory_order_relaxed);
					 remote_control_sync();
					 refresh();
				 });
		// The server restart (and the bind-retry throttle) live in
		// remote_control_sync, driven by the poll.
		QObject::connect(remote_port_spin,
				 static_cast<void (QSpinBox::*)(int)>(
					 &QSpinBox::valueChanged),
				 this, [](int value) {
					 g_device.remote_port.store(
						 value,
						 std::memory_order_relaxed);
				 });
		// Both rotation entry points (clicking the QR while waiting,
		// the disconnect button while connected) are the same token
		// swap - a plain "kick" would lose to the page's 1.2 s
		// auto-reconnect, so disconnecting implies a new QR.
		remote_qr_label->on_click = [this]() {
			remote_control_rotate_token();
			refresh();
		};
		QObject::connect(remote_kick_button, &QPushButton::clicked,
				 this, [this]() {
					 remote_control_rotate_token();
					 refresh();
				 });
		QObject::connect(mag_yaw_box, &QAbstractButton::toggled, this,
				 [](bool checked) { manager_set_mag_yaw(&g_device, checked); });
		QObject::connect(debug_box, &QAbstractButton::toggled, this, [](bool checked) {
			g_device.debug_log.store(checked, std::memory_order_relaxed);
		});

		timer = new QTimer(this);
		QObject::connect(timer, &QTimer::timeout, this, [this]() { refresh(); });
		timer->start(500);
		refresh();
	}

	QSize sizeHint() const override
	{
		return QSize(320, 520);
	}

	QSize minimumSizeHint() const override
	{
		return QSize(220, 140);
	}

private:
	// One slider unit must equal the matching spin box singleStep
	// (scale = 1 / singleStep), so dragging the slider moves the value in
	// the same increments as the spin arrows. The spin box still accepts
	// finer values typed by hand.
	static constexpr int PREDICTION_SLIDER_SCALE = 1; // step 1 ms
	static constexpr int FOV_SLIDER_SCALE = 1;        // step 1 deg
	static constexpr int BRIGHTNESS_SLIDER_SCALE = 1; // step 1 level
	static constexpr int DISTANCE_SLIDER_SCALE = 10;  // step 0.1 m
	static constexpr int SIZE_SLIDER_SCALE = 20;      // step 0.05 x
	static constexpr int CURVE_SLIDER_SCALE = 20;     // step 0.05
	static constexpr int IPD_SLIDER_SCALE = 2;        // step 0.5 mm

	static int slider_value(double value, int scale)
	{
		return static_cast<int>(std::lround(value * static_cast<double>(scale)));
	}

	static QWidget *make_double_slider(QWidget *parent, QDoubleSpinBox *spin,
					   QSlider **slider_out, int scale)
	{
		auto *row = new QWidget(parent);
		auto *layout = new QHBoxLayout(row);
		layout->setContentsMargins(0, 0, 0, 0);
		layout->setSpacing(6);

		auto *slider = new NoWheelSlider(Qt::Horizontal, row);
		slider->setRange(slider_value(spin->minimum(), scale),
				 slider_value(spin->maximum(), scale));
		slider->setSingleStep(std::max(1, slider_value(spin->singleStep(), scale)));
		slider->setPageStep(std::max(slider->singleStep(),
					     slider_value(spin->singleStep() * 10.0, scale)));
		slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
		spin->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);

		layout->addWidget(slider, 1);
		layout->addWidget(spin);
		*slider_out = slider;

		QObject::connect(slider, &QSlider::valueChanged, row,
				 [spin, scale](int value) {
					 spin->setValue(static_cast<double>(value) /
							static_cast<double>(scale));
				 });
		QObject::connect(spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 row, [slider, scale](double value) {
					 const int next = slider_value(value, scale);
					 if (slider->value() == next)
						 return;
					 QSignalBlocker block(slider);
					 slider->setValue(next);
				 });
		return row;
	}

	static void set_spin(QDoubleSpinBox *spin, double value)
	{
		if (spin->hasFocus())
			return;
		QSignalBlocker block(spin);
		spin->setValue(value);
	}

	static void set_spin(QSpinBox *spin, int value)
	{
		if (spin->hasFocus())
			return;
		QSignalBlocker block(spin);
		spin->setValue(value);
	}

	static void set_double_control(QDoubleSpinBox *spin, QSlider *slider,
				       int scale, double value)
	{
		set_spin(spin, value);
		if (slider && !slider->hasFocus() && !slider->isSliderDown()) {
			QSignalBlocker block(slider);
			slider->setValue(slider_value(value, scale));
		}
	}

	static void set_double_enabled(QDoubleSpinBox *spin, QSlider *slider, bool enabled)
	{
		spin->setEnabled(enabled);
		if (slider)
			slider->setEnabled(enabled);
	}

	void refresh()
	{
		// Apply externally-changed fold state (settings load); user
		// toggles update last_collapsed_seen themselves.
		const uint32_t collapsed =
			g_device.dock_collapsed.load(std::memory_order_relaxed);
		if (collapsed != last_collapsed_seen) {
			last_collapsed_seen = collapsed;
			status_section->set_expanded(
				!(collapsed & DOCK_SECTION_STATUS));
			device_section->set_expanded(
				!(collapsed & DOCK_SECTION_DEVICE));
			output_section->set_expanded(
				!(collapsed & DOCK_SECTION_OUTPUT));
			screen_section->set_expanded(
				!(collapsed & DOCK_SECTION_SCREEN));
			remote_section->set_expanded(
				!(collapsed & DOCK_SECTION_REMOTE));
			advanced_section->set_expanded(
				!(collapsed & DOCK_SECTION_ADVANCED));
		}

		const model_id detected = detected_hid_model(&g_device);
		const bool connected = g_device.connected.load(std::memory_order_relaxed);
		const bool enabled = g_device.connect_enabled.load(std::memory_order_relaxed);
		const bool fov_auto = g_device.fov_auto.load(std::memory_order_relaxed);
		const double fov = clampd(g_device.fov_deg.load(std::memory_order_relaxed), 20.0,
					  100.0);
		const double distance =
			clampd(g_device.screen_distance_m.load(std::memory_order_relaxed),
			       MIN_SCREEN_DISTANCE_M, MAX_SCREEN_DISTANCE_M);
		const double size_factor =
			clampd(g_device.screen_size_factor.load(std::memory_order_relaxed),
			       0.05, 4.0);
		const double screen_curve =
			clampd(g_device.screen_curve.load(std::memory_order_relaxed), 0.0,
			       MAX_SCREEN_CURVE);
		const double ipd =
			clampd(g_device.ipd_mm.load(std::memory_order_relaxed),
			       MIN_IPD_MM, MAX_IPD_MM);

		pose_snapshot p;
		{
			std::lock_guard<std::mutex> lk(g_device.state_mutex);
			p = g_device.pose;
		}

		hid_label->setText(detected == MODEL_UNKNOWN
					   ? nyan_text("detected_device.none")
					   : QString::fromStdString(
						     profile_for(detected).name));
		// Transport-specific rows (currently the One-family TCP endpoint)
		// follow the detected device; nothing detected hides them all.
		const model_profile &prof = profile_for(detected);
		const imu_transport transport = prof.transport;
		transport_label->setText(
			nyan_text(traits_for(transport).name_key));
		// Recompute row visibility when the detected MODEL changes, not just
		// the transport: feature support differs between models on the same
		// transport (e.g. BT-40 has brightness + convergence, BT-30C has
		// neither, both sensor_api), so a transport-only check would miss it.
		if (detected != last_visibility_model) {
			last_visibility_model = detected;
			const transport_traits tr = traits_for(transport);
			advanced_form->setRowVisible(ip_edit,
						     tr.uses_network_endpoint);
			advanced_form->setRowVisible(port_spin,
						     tr.uses_network_endpoint);
			// Brightness, auto-brightness and convergence are per-model
			// (profile flags), not per-transport. BT-30C has manual
			// brightness but no auto mode; BT-40 has all three.
			device_form->setRowVisible(brightness_row,
						   prof.display_brightness);
			device_form->setRowVisible(autobright_row,
						   prof.display_autobright);
			device_form->setRowVisible(convergence_row,
						   prof.display_distance);
			// Display-mode choices follow the detected family.
			{
				QSignalBlocker block(display_mode_combo);
				display_mode_combo->clear();
				for (size_t i = 0; i < tr.display_mode_count; ++i)
					display_mode_combo->addItem(
						nyan_text(
							tr.display_modes[i]
								.label_key),
						tr.display_modes[i].value);
			}
			device_form->setRowVisible(display_mode_combo,
						   tr.display_mode_count > 0);
			device_form->setRowVisible(eye_label, tr.eye_camera);
			device_form->setRowVisible(eye_button, tr.eye_camera);
			// The whole section disappears when the model has no
			// hardware controls (or nothing is detected).
			device_section->setVisible(prof.display_brightness ||
						   prof.display_autobright ||
						   prof.display_distance ||
						   tr.display_mode_count > 0 ||
						   tr.eye_camera);
		}
		// Eye camera state: adjustable while the session has the One's
		// control HID open and the Eye is attached.
		{
			const int eye_present = g_device.eye_present.load(
				std::memory_order_relaxed);
			const int eye_uvc =
				g_device.eye_uvc.load(std::memory_order_relaxed);
			const bool eye_pending =
				g_device.eye_request.load(
					std::memory_order_relaxed) >= 0;
			const char *eye_text;
			if (eye_pending)
				eye_text = nyan_text("dock.eye.switching");
			else if (eye_present < 0)
				eye_text = nyan_text("dock.eye.unknown");
			else if (eye_present == 0)
				eye_text = nyan_text("dock.eye.absent");
			else
				eye_text = nyan_text(
					eye_uvc == 1 ? "dock.eye.uvc_on"
						     : "dock.eye.uvc_off");
			eye_label->setText(eye_text);
			eye_button->setText(nyan_text(
				eye_uvc == 1 ? "dock.eye.disable"
					     : "dock.eye.enable"));
			eye_button->setEnabled(eye_present == 1 &&
					       eye_uvc >= 0 && !eye_pending);
		}
		// The display-mode row is adjustable while the session has the
		// device's command channel open (-1 = unknown/unavailable).
		const int display_mode =
			g_device.display_mode_current.load(std::memory_order_relaxed);
		display_mode_combo->setEnabled(display_mode >= 0);
		if (display_mode >= 0 &&
		    g_device.display_mode_request.load(std::memory_order_relaxed) <
			    0 &&
		    !display_mode_combo->hasFocus()) {
			const int idx = display_mode_combo->findData(display_mode);
			// Unknown values (e.g. a mode set by another tool) keep
			// the previous selection rather than picking a wrong one.
			if (idx >= 0 && idx != display_mode_combo->currentIndex()) {
				QSignalBlocker block(display_mode_combo);
				display_mode_combo->setCurrentIndex(idx);
			}
		}
		{
			const int sbs =
				g_device.sbs_output.load(std::memory_order_relaxed);
			const int idx = sbs_combo->findData(sbs);
			if (idx >= 0 && idx != sbs_combo->currentIndex() &&
			    !sbs_combo->hasFocus()) {
				QSignalBlocker block(sbs_combo);
				sbs_combo->setCurrentIndex(idx);
			}
		}
		// Brightness is only adjustable while the session has the
		// serial command port open (-1 = unknown/unavailable) and the
		// device is not driving it from its ambient light sensor.
		const int brightness =
			g_device.brightness_current.load(std::memory_order_relaxed);
		const int autobright =
			g_device.autobright_current.load(std::memory_order_relaxed);
		autobright_box->setEnabled(autobright >= 0);
		if (g_device.autobright_request.load(std::memory_order_relaxed) <
		    0) {
			QSignalBlocker block(autobright_box);
			autobright_box->setChecked(autobright == 1);
		}
		set_double_enabled(brightness_spin, brightness_slider,
				   brightness >= 0 && autobright != 1);
		if (brightness >= 0 && brightness <= 20 &&
		    g_device.brightness_request.load(std::memory_order_relaxed) < 0)
			set_double_control(brightness_spin, brightness_slider,
					   BRIGHTNESS_SLIDER_SCALE, brightness);
		// Convergence link is actionable only while the session has
		// the command port open on a model with setdisplaydistance
		// (BT-40; the BT-30C lacks the command and stays grayed out).
		convergence_box->setEnabled(
			g_device.display_distance_current.load(
				std::memory_order_relaxed) != INT32_MIN);
		{
			QSignalBlocker block(convergence_box);
			convergence_box->setChecked(g_device.convergence_link.load(
				std::memory_order_relaxed));
		}
		stream_label->setText(!enabled ? nyan_text("dock.stream.disabled")
					       : (connected
							  ? nyan_text("dock.stream.connected")
							  : nyan_text("dock.stream.waiting")));
		const char *pose_status = nyan_text("dock.pose.disconnected");
		if (!enabled) {
			pose_status = nyan_text("dock.pose.disabled");
		} else if (connected && p.connected) {
			pose_status = p.calibrated
					      ? nyan_text("dock.pose.calibrated")
					      : nyan_text("dock.pose.calibrating");
		}
		pose_label->setText(pose_status);
		// Recenter / recalibrate need a live IMU; gray them out while the
		// glasses are not connected (matches the "disconnected" pose state).
		recenter->setEnabled(connected);
		recalibrate->setEnabled(connected);
		// Collapsed-status summary: green = tracking (calibrated),
		// yellow = connecting/calibrating, red = no device or follow
		// off. The tooltip carries the textual state so the color is
		// never the only signal.
		{
			const char *light = "#d9534f";
			if (enabled && detected != MODEL_UNKNOWN)
				light = (connected && p.connected &&
					 p.calibrated)
						? "#5cb85c"
						: "#f0ad4e";
			const QString model_name =
				detected == MODEL_UNKNOWN
					? QString::fromUtf8(nyan_text(
						  "detected_device.none"))
					: QString::fromStdString(
						  profile_for(detected).name);
			status_section->set_summary(
				QStringLiteral(
					"<span style=\"color:%1;\">●</span> %2")
					.arg(QString::fromLatin1(light),
					     model_name.toHtmlEscaped()),
				QString::fromUtf8(pose_status));
		}
		const int virtual_count =
			g_device.virtual_source_count.load(std::memory_order_relaxed);
		virtual_label->setText(QString::number(virtual_count));
		// The auto-fullscreen latch is per glasses connection, but a
		// virtual screen source appearing (added to the scene while
		// the glasses are already connected) should fire it too.
		if (last_virtual_count == 0 && virtual_count > 0)
			auto_projector_opened = false;
		last_virtual_count = virtual_count;

		nyan_real_glasses_display_info glasses;
		const bool glasses_display_present =
			nyan_real_find_glasses_display(&glasses);
		const bool glasses_rect_valid =
			glasses_display_present && glasses.has_rect &&
			glasses.width > 0 && glasses.height > 0;
		g_glasses_display_width.store(glasses_rect_valid ? glasses.width
								 : 0,
					      std::memory_order_relaxed);
		g_glasses_display_height.store(glasses_rect_valid
						       ? glasses.height
						       : 0,
					       std::memory_order_relaxed);
		// The cursor fence is a system-wide LL mouse hook tied to the glasses
		// display, so it must release when the app is disabled (master off =
		// glasses freed for other use), not just when the box is unchecked.
		cursor_fence_update(
			g_device.cursor_fence.load(std::memory_order_relaxed) &&
				g_device.connect_enabled.load(
					std::memory_order_relaxed),
			glasses_rect_valid, glasses.x, glasses.y,
			glasses.x + static_cast<long>(glasses.width),
			glasses.y + static_cast<long>(glasses.height));
		// IPD only affects SBS rendering; gray the row out otherwise.
		// Same output-size fallback as virtual_source_tick, so the row's
		// state matches what the renderer actually does (including
		// manual SBS ON without a glasses display).
		{
			const model_profile &profile = profile_for(detected);
			const uint32_t out_w = glasses_rect_valid
						       ? glasses.width
						       : profile.display_width;
			const uint32_t out_h = glasses_rect_valid
						       ? glasses.height
						       : profile.display_height;
			set_double_enabled(ipd_spin, ipd_slider,
					   sbs_output_active(out_w, out_h));
		}
		glasses_display_label->setText(
			glasses_display_present
				? QString::fromStdString(
					  glasses.friendly_name.empty()
						  ? glasses.gdi_device
						  : glasses.friendly_name)
				: QString(nyan_text(
					  "dock.glasses_display.none")));
		// Disabled (not hidden) so the feature stays discoverable; the
		// "glasses display: not detected" status row explains why. The
		// auto-fullscreen checkbox below stays interactive because it is
		// a pre-arm setting for the next connection.
		projector_button->setEnabled(glasses_display_present);
		{
			QSignalBlocker block(auto_projector_box);
			auto_projector_box->setChecked(g_device.auto_projector.load(
				std::memory_order_relaxed));
		}
		// Requests from the phone remote's settings mirror that need
		// the dock's Qt context (remote_schema rows park them on
		// g_device; same consume pattern as brightness_request).
		if (g_device.projector_request.exchange(
			    false, std::memory_order_relaxed)) {
			if (open_glasses_source_projector(true))
				auto_projector_opened = true;
		}
		if (g_device.monitor_rearm.exchange(false,
						    std::memory_order_relaxed)) {
			auto_monitor_applied = false;
			monitor_device_applied = false;
		}
		refresh_monitor_combo();
		const int monitor_out =
			g_device.monitor_out.load(std::memory_order_relaxed);
		if (monitor_out == MONITOR_OUT_AUTO_GLASSES) {
			// One auto-switch latch per appearance of the glasses'
			// audio endpoint. USB audio can show up later than HID
			// (retry every poll until it exists), and it can also
			// drop and re-enumerate while HID stays connected
			// (seen on hardware: the endpoint vanished mid-session
			// and the monitors died with it - libobs never retries
			// a lost monitor). The latch therefore re-arms whenever
			// the endpoint is absent; the re-application on return
			// rebuilds all monitors.
			if (detected == MODEL_UNKNOWN) {
				auto_monitor_applied = false;
			} else {
				const monitoring_device_match m =
					find_glasses_monitoring_device();
				if (!m.found)
					auto_monitor_applied = false;
				else if (!auto_monitor_applied &&
					 apply_monitoring_device(
						 m, "glasses detected"))
					auto_monitor_applied = true;
			}
		} else if (monitor_out == MONITOR_OUT_DEVICE) {
			// Hold monitoring on the chosen endpoint. While it is
			// absent (Bluetooth powered off) the choice stays put
			// and OBS monitoring is left untouched; the latch
			// re-arms so the endpoint is re-applied the moment it
			// enumerates again.
			std::string want_id;
			{
				std::lock_guard<std::mutex> lk(
					g_device.settings_mutex);
				want_id = g_device.monitor_device_id;
			}
			const monitoring_device_match m =
				find_monitoring_device_by_id(want_id);
			if (!m.found) {
				monitor_device_applied = false;
			} else if (!monitor_device_applied &&
				   apply_monitoring_device(m,
							   "selected output")) {
				monitor_device_applied = true;
				// Endpoint names drift with connection state;
				// keep the stored display name current.
				std::lock_guard<std::mutex> lk(
					g_device.settings_mutex);
				g_device.monitor_device_name = m.name;
			}
		}
		{
			QSignalBlocker block(cursor_fence_box);
			cursor_fence_box->setChecked(g_device.cursor_fence.load(
				std::memory_order_relaxed));
		}
		refresh_volume();
		if (!glasses_display_present) {
			// Close the glasses output and re-arm the auto-open for
			// the next connection (the host tears down the windows it
			// tracked while the display was present).
			nyan_close_glasses_output();
			auto_projector_opened = false;
		} else {
			if (g_device.auto_projector.load(
				    std::memory_order_relaxed) &&
			    !auto_projector_opened && detected != MODEL_UNKNOWN &&
			    virtual_count > 0) {
				if (open_glasses_source_projector(false))
					auto_projector_opened = true;
			}
			// Let the host snapshot the glasses-screen output so it
			// can be closed when the display disappears.
			nyan_track_glasses_output(projector_monitor_index(glasses));
		}

		{
			// Eye-icon now reflects head-pose follow (3DoF), not connect.
			QSignalBlocker block(connect_box);
			connect_box->setChecked(
				g_device.pose_follow.load(std::memory_order_relaxed));
		}
		if (!ip_edit->hasFocus()) {
			std::lock_guard<std::mutex> lk(g_device.settings_mutex);
			QSignalBlocker block(ip_edit);
			ip_edit->setText(QString::fromStdString(g_device.ip));
			set_spin(port_spin, g_device.port);
		}
		{
			QSignalBlocker block(fov_auto_box);
			fov_auto_box->setChecked(fov_auto);
		}
		set_double_enabled(fov_spin, fov_slider, !fov_auto);
		set_double_control(prediction_spin, prediction_slider,
				   PREDICTION_SLIDER_SCALE,
				   g_device.prediction_ms.load(std::memory_order_relaxed));
		set_double_control(fov_spin, fov_slider, FOV_SLIDER_SCALE, fov);
		set_double_control(distance_spin, distance_slider, DISTANCE_SLIDER_SCALE,
				   distance);
		set_double_control(size_spin, size_slider, SIZE_SLIDER_SCALE, size_factor);
		set_double_control(curve_spin, curve_slider, CURVE_SLIDER_SCALE,
				   screen_curve);
		set_double_control(ipd_spin, ipd_slider, IPD_SLIDER_SCALE, ipd);
		{
			QSignalBlocker block(offscreen_box);
			offscreen_box->setChecked(g_device.offscreen_indicator.load(
				std::memory_order_relaxed));
		}
		{
			// Keep the shown focus in sync (remote/hotkey can change it);
			// repopulate happens on popup, so just select the number here.
			const int focus = g_device.focus_display.load(
				std::memory_order_relaxed);
			QSignalBlocker block(focus_combo);
			int idx = focus_combo->findData(focus);
			focus_combo->setCurrentIndex(idx >= 0 ? idx : 0);
		}
		{
			QSignalBlocker block(mag_yaw_box);
			mag_yaw_box->setChecked(g_device.mag_yaw.load(std::memory_order_relaxed));
		}
		{
			QSignalBlocker block(debug_box);
			debug_box->setChecked(g_device.debug_log.load(std::memory_order_relaxed));
		}
		refresh_remote();

		const double diag_m =
			2.0 * SCREEN_SIZE_UNIT_DISTANCE_M * std::tan(fov * PI / 360.0) *
			size_factor;
		const double diag_in = diag_m / 0.0254;
		const double apparent_fov = 2.0 * std::atan(diag_m / (2.0 * distance)) *
					    180.0 / PI;
		screen_label->setText(QString::asprintf("%.1f in / %.1f deg", diag_in,
							apparent_fov));

		// SBS comfort guidance: the distance tooltip names the active
		// model's optical focal distance - setting the screen distance
		// there makes vergence match accommodation, which is easiest
		// on the eyes during long SBS sessions.
		const float focus_m = profile_for(detected).optics_focus();
		if (focus_m != last_focus_tip_m) {
			last_focus_tip_m = focus_m;
			distance_row->setToolTip(QStringLiteral("<qt>%1 %2</qt>").arg(
				QString::fromUtf8(nyan_text(
					"screen_distance_tooltip")),
				QString::asprintf(
					nyan_text(
						"screen_distance_focus_note"),
					focus_m)));
		}
	}

	// Reconcile the phone-remote server with the dock state, then mirror
	// the result: a "connected" line while a phone is on, a QR code + URL
	// while it waits for one, a hint while the LAN address (or the port)
	// is unavailable, nothing while disabled.
	void refresh_remote()
	{
		remote_control_sync();
		const bool enabled =
			g_device.remote_enabled.load(std::memory_order_relaxed);
		{
			QSignalBlocker block(remote_enable_box);
			remote_enable_box->setChecked(enabled);
		}
		set_spin(remote_port_spin,
			 g_device.remote_port.load(std::memory_order_relaxed));
		const int clients = enabled ? remote_control_client_count() : 0;
		const std::string url =
			(enabled && clients == 0) ? remote_control_url() : "";
		if (url == last_remote_url && enabled == last_remote_enabled &&
		    clients == last_remote_clients)
			return;
		last_remote_url = url;
		last_remote_enabled = enabled;
		last_remote_clients = clients;
		// Collapsed-header summary, mirroring the status section's
		// traffic light: green = a phone is connected, yellow =
		// waiting for a scan, red = server failed to start.
		const auto summarize = [this](const char *light,
					      const QString &text) {
			remote_section->set_summary(
				QStringLiteral(
					"<span style=\"color:%1;\">●</span> %2")
					.arg(QString::fromLatin1(light),
					     text.toHtmlEscaped()),
				text);
		};
		if (clients > 0) {
			// A phone is connected: the QR has served its purpose,
			// show the session state instead.
			remote_qr_label->clear();
			remote_qr_label->setVisible(false);
			remote_url_label->setText(QString::asprintf(
				nyan_text("dock.remote_connected"),
				clients));
			remote_url_label->setVisible(true);
			remote_kick_button->setVisible(true);
			summarize("#5cb85c",
				  QString::asprintf(
					  nyan_text(
						  "dock.remote_summary_connected"),
					  clients));
			return;
		}
		remote_kick_button->setVisible(false);
		if (url.empty()) {
			remote_qr_label->clear();
			remote_qr_label->setVisible(false);
			remote_url_label->setText(
				enabled ? nyan_text(
						  "dock.remote_unavailable")
					: "");
			remote_url_label->setVisible(enabled);
			if (enabled)
				summarize("#d9534f",
					  QString::fromUtf8(nyan_text(
						  "dock.remote_summary_error")));
			else
				remote_section->set_summary(QString(),
							    QString());
			return;
		}
		summarize("#f0ad4e", QString::fromUtf8(nyan_text(
					     "dock.remote_summary_waiting")));
		// Crisp integer-scaled QR with a quiet zone; the white pad
		// keeps it scannable on dark themes.
		const qrcodegen::QrCode qr = qrcodegen::QrCode::encodeText(
			url.c_str(), qrcodegen::QrCode::Ecc::MEDIUM);
		const int n = qr.getSize();
		const int quiet = 4;
		QImage img(n + quiet * 2, n + quiet * 2,
			   QImage::Format_RGB32);
		img.fill(Qt::white);
		for (int y = 0; y < n; y++) {
			for (int x = 0; x < n; x++) {
				if (qr.getModule(x, y))
					img.setPixel(x + quiet, y + quiet,
						     qRgb(0, 0, 0));
			}
		}
		const int scale = std::max(2, 192 / img.width());
		remote_qr_label->setPixmap(QPixmap::fromImage(
			img.scaled(img.width() * scale, img.width() * scale,
				   Qt::KeepAspectRatio,
				   Qt::FastTransformation)));
		remote_qr_label->setVisible(true);
		remote_url_label->setText(QString::fromStdString(url));
		remote_url_label->setVisible(true);
	}

	// Sync the monitoring-output combo with the present device list and
	// The WASAPI endpoint id audio is currently routed to: the glasses (auto)
	// or the user's pick. Empty for "keep" (OBS default = not ours to touch)
	// or when the device is absent. The volume controls act on this id.
	std::string current_output_id()
	{
		const int mode =
			g_device.monitor_out.load(std::memory_order_relaxed);
		if (mode == MONITOR_OUT_AUTO_GLASSES)
			return find_glasses_monitoring_device().id;
		if (mode == MONITOR_OUT_DEVICE) {
			std::lock_guard<std::mutex> lk(g_device.settings_mutex);
			return g_device.monitor_device_id;
		}
		return "";
	}

	// Reflect the live endpoint volume/mute into the widgets (poll). Disabled
	// when there is no controllable endpoint; skips the slider while the user
	// drags it so the poll cannot fight the gesture.
	void refresh_volume()
	{
		if (!volume_slider || !mute_box)
			return;
		const std::string id = current_output_id();
		const bool have = !id.empty();
		volume_slider->setEnabled(have);
		mute_box->setEnabled(have);
		if (!have)
			return;
		if (!volume_slider->isSliderDown()) {
			const float v = endpoint_volume_get(id);
			if (v >= 0.0f) {
				QSignalBlocker block(volume_slider);
				volume_slider->setValue(static_cast<int>(
					std::lround(v * 100.0f)));
			}
		}
		bool ok = false;
		const bool muted = endpoint_volume_get_mute(id, &ok);
		if (ok) {
			QSignalBlocker block(mute_box);
			mute_box->setChecked(muted);
		}
	}

	// the stored selection. itemData carries the endpoint id ("@auto" /
	// "@keep" for the modes), UserRole + 1 the raw endpoint name without
	// the absent-state suffix. Rebuilds only when the content actually
	// changed, so the poll does not disturb the combo needlessly.
	void refresh_monitor_combo()
	{
		const int mode =
			g_device.monitor_out.load(std::memory_order_relaxed);
		std::string sel_id, sel_name;
		{
			std::lock_guard<std::mutex> lk(g_device.settings_mutex);
			sel_id = g_device.monitor_device_id;
			sel_name = g_device.monitor_device_name;
		}
		struct entry_t {
			QString label;
			QString id;
			QString name;
		};
		QList<entry_t> entries;
		entries.append({QString::fromUtf8(nyan_text(
					 "dock.monitor_out.auto")),
				QStringLiteral("@auto"), QString()});
		// "Leave as-is" only means "leave OBS's monitoring device alone"; the
		// standalone manages its own output, so the host drops it (nyan_caps).
		if (nyan_caps().obs_monitoring_keep)
			entries.append({QString::fromUtf8(nyan_text(
						 "dock.monitor_out.keep")),
					QStringLiteral("@keep"), QString()});
		nyan_enum_audio_outputs(
			[](void *data, const char *name, const char *id) {
				auto *e = static_cast<QList<entry_t> *>(data);
				const QString n =
					QString::fromUtf8(name ? name : "");
				e->append({n, QString::fromUtf8(id ? id : ""),
					   n});
				return true;
			},
			&entries);
		int want_index = 0; // MONITOR_OUT_AUTO_GLASSES
		if (mode == MONITOR_OUT_KEEP) {
			want_index = 1;
		} else if (mode == MONITOR_OUT_DEVICE && !sel_id.empty()) {
			want_index = -1;
			for (int i = 2; i < entries.size(); ++i) {
				if (entries[i].id.toStdString() == sel_id) {
					want_index = i;
					break;
				}
			}
			if (want_index < 0) {
				// The remembered endpoint is absent (Bluetooth
				// powered off): keep it listed and selected
				// instead of dropping the user's choice.
				const QString name = QString::fromStdString(
					sel_name.empty() ? sel_id : sel_name);
				entries.append(
					{name + QString::fromUtf8(nyan_text(
							"dock.monitor_out.missing_suffix")),
					 QString::fromStdString(sel_id), name});
				want_index = entries.size() - 1;
			}
		}
		bool same = monitor_combo->count() == entries.size();
		for (int i = 0; same && i < entries.size(); ++i) {
			same = monitor_combo->itemText(i) == entries[i].label &&
			       monitor_combo->itemData(i).toString() ==
				       entries[i].id;
		}
		if (same) {
			if (monitor_combo->currentIndex() != want_index) {
				QSignalBlocker block(monitor_combo);
				monitor_combo->setCurrentIndex(want_index);
			}
			return;
		}
		// Rebuilding closes an open popup; retry on the next poll.
		if (monitor_combo->view()->isVisible())
			return;
		QSignalBlocker block(monitor_combo);
		monitor_combo->clear();
		for (const entry_t &e : entries) {
			monitor_combo->addItem(e.label, e.id);
			monitor_combo->setItemData(monitor_combo->count() - 1,
						   e.name, Qt::UserRole + 1);
		}
		monitor_combo->setCurrentIndex(want_index);
	}

	QLabel *hid_label = nullptr;
	QLabel *glasses_display_label = nullptr;
	QLabel *transport_label = nullptr;
	QLabel *stream_label = nullptr;
	QLabel *pose_label = nullptr;
	QLabel *virtual_label = nullptr;
	QLabel *screen_label = nullptr;
	ToggleSwitch *connect_box = nullptr;
	QFormLayout *device_form = nullptr;
	QFormLayout *advanced_form = nullptr;
	DockSection *status_section = nullptr;
	DockSection *device_section = nullptr;
	DockSection *output_section = nullptr;
	DockSection *screen_section = nullptr;
	DockSection *remote_section = nullptr;
	DockSection *advanced_section = nullptr;
	ToggleSwitch *remote_enable_box = nullptr;
	QSpinBox *remote_port_spin = nullptr;
	ClickableLabel *remote_qr_label = nullptr;
	QLabel *remote_url_label = nullptr;
	QPushButton *remote_kick_button = nullptr;
	// Last QR/URL state rendered, to skip the needless re-encode.
	std::string last_remote_url;
	bool last_remote_enabled = false;
	int last_remote_clients = 0;
	// Last dock_collapsed mask seen, to detect settings loads.
	uint32_t last_collapsed_seen = UINT32_MAX;
	QLineEdit *ip_edit = nullptr;
	QSpinBox *port_spin = nullptr;
	QDoubleSpinBox *brightness_spin = nullptr;
	QSlider *brightness_slider = nullptr;
	QWidget *brightness_row = nullptr;
	ToggleSwitch *autobright_box = nullptr;
	ToggleSwitch *convergence_box = nullptr;
	// Field wrappers for the two switches above, so setRowVisible can hide the
	// whole row by device capability (the switch is nested in the wrapper).
	QWidget *autobright_row = nullptr;
	QWidget *convergence_row = nullptr;
	QComboBox *display_mode_combo = nullptr;
	QLabel *eye_label = nullptr;
	QPushButton *eye_button = nullptr;
	// Recenter / gyro-bias recalibrate: act on the IMU, so they are disabled
	// while the glasses are not connected (refresh() toggles them).
	QPushButton *recenter = nullptr;
	QPushButton *recalibrate = nullptr;
	QComboBox *sbs_combo = nullptr;
	int last_visibility_model = -1; // model_id last applied to row visibility
	QDoubleSpinBox *prediction_spin = nullptr;
	QSlider *prediction_slider = nullptr;
	ToggleSwitch *fov_auto_box = nullptr;
	QDoubleSpinBox *fov_spin = nullptr;
	QSlider *fov_slider = nullptr;
	// Row container of the distance slider; refresh() rewrites its
	// tooltip when the detected model (and so its optical focal
	// distance) changes.
	QWidget *distance_row = nullptr;
	float last_focus_tip_m = 0.0f;
	QDoubleSpinBox *distance_spin = nullptr;
	QSlider *distance_slider = nullptr;
	QDoubleSpinBox *size_spin = nullptr;
	QSlider *size_slider = nullptr;
	QDoubleSpinBox *curve_spin = nullptr;
	QSlider *curve_slider = nullptr;
	ToggleSwitch *offscreen_box = nullptr;
	FocusComboBox *focus_combo = nullptr;
	QDoubleSpinBox *ipd_spin = nullptr;
	QSlider *ipd_slider = nullptr;
	ToggleSwitch *mag_yaw_box = nullptr;
	ToggleSwitch *debug_box = nullptr;
	QPushButton *projector_button = nullptr;
	ToggleSwitch *auto_projector_box = nullptr;
	QComboBox *monitor_combo = nullptr;
	QWidget *volume_row = nullptr;
	QSlider *volume_slider = nullptr;
	ToggleSwitch *mute_box = nullptr;
	ToggleSwitch *cursor_fence_box = nullptr;
	// Auto-open latch: one projector per glasses-display connection,
	// re-armed when a virtual screen source first appears.
	bool auto_projector_opened = false;
	int last_virtual_count = -1;
	// Monitoring-device latches: auto mode arms once per detected
	// connection, device mode once per endpoint appearance (re-armed when
	// the endpoint disappears so its return re-applies the choice).
	bool auto_monitor_applied = false;
	bool monitor_device_applied = false;
	QTimer *timer = nullptr;
};

// Creates the dock widget. The host shell registers it (OBS frontend dock or
// the standalone's window); shared UI logic lives entirely in NyanRealDock.
QWidget *create_nyan_real_dock(QWidget *parent)
{
	return new NyanRealDock(parent);
}
#endif
