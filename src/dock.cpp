// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
#include "dock.h"

#include <windows.h>

#include <obs-frontend-api.h>
#include <obs-module.h>

#ifdef NYAN_REAL_3DOF_WITH_QT_DOCK
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QDockWidget>
#include <QFormLayout>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPointer>
#include <QPushButton>
#include <QScreen>
#include <QSignalBlocker>
#include <QSlider>
#include <QScrollArea>
#include <QSize>
#include <QSizePolicy>
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
#include <string>

#include "cursor_fence.h"
#include "device_manager.h"
#include "device_registry.h"
#include "display-wall-source.h"
#include "virtual_source.h"

#ifdef NYAN_REAL_3DOF_WITH_QT_DOCK
// First nyan Real virtual screen source, used as the projector content.
static bool find_virtual_source_name(std::string &name_out)
{
	struct ctx_t {
		std::string name;
		bool found = false;
	} ctx;
	obs_enum_sources(
		[](void *param, obs_source_t *src) {
			auto *c = static_cast<ctx_t *>(param);
			if (!is_virtual_source_id(obs_source_get_id(src)) ||
			    obs_source_removed(src))
				return true;
			const char *n = obs_source_get_name(src);
			if (!n || !*n)
				return true;
			c->name = n;
			c->found = true;
			return false;
		},
		&ctx);
	if (ctx.found)
		name_out = ctx.name;
	return ctx.found;
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

// Projector windows are OBS's own QWidget top-levels of class OBSProjector.
static QList<QWidget *> projectors_on_screen(QScreen *screen)
{
	QList<QWidget *> out;
	if (!screen)
		return out;
	const QList<QWidget *> widgets = QApplication::topLevelWidgets();
	for (QWidget *widget : widgets) {
		if (strcmp(widget->metaObject()->className(), "OBSProjector") !=
		    0)
			continue;
		if (widget->screen() != screen)
			continue;
		out.append(widget);
	}
	return out;
}

// Close existing projector windows on the target screen so the dock button
// and the auto-open never stack projectors. obs_frontend_open_projector opens
// a new window on every call, and OBS only deduplicates when the user enabled
// "Limit one fullscreen projector per screen".
static void close_projectors_on_screen(QScreen *screen)
{
	for (QWidget *widget : projectors_on_screen(screen))
		widget->close();
}

// Opens the virtual screen's fullscreen source projector on the glasses
// display. UI thread only.
static bool open_glasses_source_projector(bool log_failure)
{
	nyan_real_glasses_display_info display;
	if (!nyan_real_find_glasses_display(&display)) {
		if (log_failure)
			blog(LOG_WARNING,
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
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] glasses display %s ('%s') not matched; Qt screens: %s",
			     display.gdi_device.c_str(),
			     display.friendly_name.c_str(),
			     screen_names.c_str());
		}
		return false;
	}
	std::string source_name;
	if (!find_virtual_source_name(source_name)) {
		if (log_failure)
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] no virtual screen source exists; add one before opening the projector");
		return false;
	}
	close_projectors_on_screen(QGuiApplication::screens().value(monitor));
	obs_frontend_open_projector("Source", monitor, nullptr,
				    source_name.c_str());
	blog(LOG_INFO,
	     "[obs-nyan-real-3dof] opened source projector '%s' on %s (%s, monitor %d)",
	     source_name.c_str(), display.friendly_name.c_str(),
	     display.gdi_device.c_str(), monitor);
	return true;
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

// Switch OBS's audio monitoring device to the glasses' USB audio endpoint.
// Returns true once monitoring points at the glasses (already or newly set);
// false while no glasses audio endpoint is present yet (USB audio can
// enumerate later than HID, so the caller retries on its poll).
static bool apply_glasses_monitoring_device()
{
	struct match_t {
		std::string name;
		std::string id;
		bool found = false;
	} m;
	obs_enum_audio_monitoring_devices(
		[](void *data, const char *name, const char *id) {
			auto *m = static_cast<match_t *>(data);
			if (!is_glasses_audio_name(name))
				return true;
			m->name = name ? name : "";
			m->id = id ? id : "";
			m->found = true;
			return false;
		},
		&m);
	if (!m.found)
		return false;
	const char *cur_name = nullptr;
	const char *cur_id = nullptr;
	obs_get_audio_monitoring_device(&cur_name, &cur_id);
	if (cur_id && m.id == cur_id) {
		// Already the configured device - but monitors created while
		// the USB endpoint was still enumerating (OBS launch racing
		// the glasses) failed with AUDCLNT_E_DEVICE_INVALIDATED, and
		// OBS never retries them on its own. The endpoint provably
		// exists right now (it was just enumerated), so rebuild all
		// monitors against it. Runs once per glasses connection.
		obs_reset_audio_monitoring();
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] audio monitoring re-initialized ('%s' is ready)",
		     m.name.c_str());
		return true;
	}
	if (!obs_set_audio_monitoring_device(m.name.c_str(), m.id.c_str()))
		return false;
	blog(LOG_INFO,
	     "[obs-nyan-real-3dof] audio monitoring device -> '%s' (glasses detected)",
	     m.name.c_str());
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

		auto *action_row_1 = new QHBoxLayout();
		auto *recenter = new QPushButton(obs_module_text("recenter"), content);
		auto *recalibrate = new QPushButton(obs_module_text("recalibrate"), content);
		recenter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
		recalibrate->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
		action_row_1->addWidget(recenter);
		action_row_1->addWidget(recalibrate);
		root->addLayout(action_row_1);


		auto *status_group = new QGroupBox(obs_module_text("dock.status"), content);
		auto *status_form = new QFormLayout(status_group);
		hid_label = new QLabel(status_group);
		glasses_display_label = new QLabel(status_group);
		transport_label = new QLabel(status_group);
		stream_label = new QLabel(status_group);
		pose_label = new QLabel(status_group);
		virtual_label = new QLabel(status_group);
		status_form->addRow(obs_module_text("dock.hid"), hid_label);
		status_form->addRow(obs_module_text("dock.glasses_display"),
				    glasses_display_label);
		status_form->addRow(obs_module_text("dock.transport"), transport_label);
		status_form->addRow(obs_module_text("dock.stream"), stream_label);
		status_form->addRow(obs_module_text("dock.pose"), pose_label);
		status_form->addRow(obs_module_text("dock.virtual_sources"),
				    virtual_label);
		root->addWidget(status_group);

		auto *device_group = new QGroupBox(obs_module_text("dock.device"), content);
		device_form = new QFormLayout(device_group);
		connect_box = new QCheckBox(obs_module_text("connect_enabled"), device_group);
		ip_edit = new QLineEdit(device_group);
		port_spin = new NoWheelSpinBox(device_group);
		port_spin->setRange(1, 65535);
		brightness_spin = new NoWheelDoubleSpinBox(device_group);
		brightness_spin->setRange(0.0, 20.0);
		brightness_spin->setDecimals(0);
		brightness_spin->setSingleStep(1.0);
		device_form->addRow(QString(), connect_box);
		device_form->addRow(obs_module_text("ip"), ip_edit);
		device_form->addRow(obs_module_text("port"), port_spin);
		brightness_row = make_double_slider(device_group, brightness_spin,
						    &brightness_slider,
						    BRIGHTNESS_SLIDER_SCALE);
		brightness_row->setToolTip(
			obs_module_text("brightness_tooltip"));
		device_form->addRow(obs_module_text("brightness"),
				    brightness_row);
		autobright_box = new QCheckBox(obs_module_text("autobright"),
					       device_group);
		autobright_box->setToolTip(
			obs_module_text("autobright_tooltip"));
		device_form->addRow(QString(), autobright_box);
		display_mode_combo = new NoWheelComboBox(device_group);
		display_mode_combo->setToolTip(
			obs_module_text("displaymode_tooltip"));
		device_form->addRow(obs_module_text("displaymode"),
				    display_mode_combo);
		eye_label = new QLabel(device_group);
		device_form->addRow(obs_module_text("dock.eye"), eye_label);
		eye_button = new QPushButton(device_group);
		eye_button->setToolTip(obs_module_text("dock.eye_tooltip"));
		device_form->addRow(QString(), eye_button);
		dist_marker_box = new QCheckBox(
			obs_module_text("screen_dist_marker"), device_group);
		dist_marker_box->setToolTip(
			obs_module_text("screen_dist_marker_tooltip"));
		device_form->addRow(QString(), dist_marker_box);
		sbs_combo = new NoWheelComboBox(device_group);
		sbs_combo->addItem(obs_module_text("sbs_output.auto"), 0);
		sbs_combo->addItem(obs_module_text("sbs_output.on"), 1);
		sbs_combo->addItem(obs_module_text("sbs_output.off"), 2);
		sbs_combo->setToolTip(obs_module_text("sbs_output_tooltip"));
		device_form->addRow(obs_module_text("sbs_output"), sbs_combo);
		projector_button = new QPushButton(
			obs_module_text("dock.open_projector"), device_group);
		projector_button->setToolTip(
			obs_module_text("dock.open_projector_tooltip"));
		auto_projector_box = new QCheckBox(
			obs_module_text("dock.auto_projector"), device_group);
		auto_projector_box->setToolTip(
			obs_module_text("dock.auto_projector_tooltip"));
		auto_monitor_box = new QCheckBox(
			obs_module_text("dock.auto_monitor"), device_group);
		auto_monitor_box->setToolTip(
			obs_module_text("dock.auto_monitor_tooltip"));
		cursor_fence_box = new QCheckBox(
			obs_module_text("dock.cursor_fence"), device_group);
		cursor_fence_box->setToolTip(
			obs_module_text("dock.cursor_fence_tooltip"));
		device_form->addRow(QString(), projector_button);
		device_form->addRow(QString(), auto_projector_box);
		device_form->addRow(QString(), auto_monitor_box);
		device_form->addRow(QString(), cursor_fence_box);
		root->addWidget(device_group);

		auto *screen_group = new QGroupBox(obs_module_text("dock.screen"), content);
		auto *screen_form = new QFormLayout(screen_group);
		prediction_spin = new NoWheelDoubleSpinBox(screen_group);
		prediction_spin->setRange(0.0, 50.0);
		prediction_spin->setDecimals(0);
		prediction_spin->setSingleStep(1.0);
		fov_auto_box = new QCheckBox(obs_module_text("fov_auto"), screen_group);
		fov_spin = new NoWheelDoubleSpinBox(screen_group);
		fov_spin->setRange(20.0, 100.0);
		fov_spin->setDecimals(0);
		fov_spin->setSingleStep(1.0);
		distance_spin = new NoWheelDoubleSpinBox(screen_group);
		distance_spin->setRange(MIN_SCREEN_DISTANCE_M,
					MAX_SCREEN_DISTANCE_M);
		distance_spin->setDecimals(1);
		distance_spin->setSingleStep(0.1);
		size_spin = new NoWheelDoubleSpinBox(screen_group);
		// Down to 0.05: the marker distance sync scales the factor
		// with the distance ratio, and a 4 m -> 0.5 m move needs 1/8.
		size_spin->setRange(0.05, 4.0);
		size_spin->setDecimals(2);
		size_spin->setSingleStep(0.05);
		curve_spin = new NoWheelDoubleSpinBox(screen_group);
		curve_spin->setRange(0.0, MAX_SCREEN_CURVE);
		curve_spin->setDecimals(2);
		curve_spin->setSingleStep(0.05);
		ipd_spin = new NoWheelDoubleSpinBox(screen_group);
		ipd_spin->setRange(MIN_IPD_MM, MAX_IPD_MM);
		ipd_spin->setDecimals(1);
		ipd_spin->setSingleStep(0.5);
		screen_label = new QLabel(screen_group);
		screen_form->addRow(obs_module_text("prediction_ms"),
				    make_double_slider(screen_group, prediction_spin,
						       &prediction_slider,
						       PREDICTION_SLIDER_SCALE));
		screen_form->addRow(QString(), fov_auto_box);
		screen_form->addRow(obs_module_text("fov_deg"),
				    make_double_slider(screen_group, fov_spin, &fov_slider,
						       FOV_SLIDER_SCALE));
		auto *distance_row = make_double_slider(screen_group, distance_spin,
							&distance_slider,
							DISTANCE_SLIDER_SCALE);
		distance_row->setToolTip(
			obs_module_text("screen_distance_tooltip"));
		screen_form->addRow(obs_module_text("screen_distance_m"),
				    distance_row);
		screen_form->addRow(obs_module_text("screen_size_factor"),
				    make_double_slider(screen_group, size_spin, &size_slider,
						       SIZE_SLIDER_SCALE));
		screen_form->addRow(obs_module_text("screen_curve"),
				    make_double_slider(screen_group, curve_spin, &curve_slider,
						       CURVE_SLIDER_SCALE));
		auto *ipd_row = make_double_slider(screen_group, ipd_spin,
						   &ipd_slider, IPD_SLIDER_SCALE);
		ipd_row->setToolTip(obs_module_text("ipd_tooltip"));
		screen_form->addRow(obs_module_text("ipd_mm"), ipd_row);
		screen_form->addRow(obs_module_text("dock.screen_result"), screen_label);
		root->addWidget(screen_group);

		auto *options_group = new QGroupBox(obs_module_text("dock.options"), content);
		auto *options_layout = new QVBoxLayout(options_group);
		mag_yaw_box = new QCheckBox(obs_module_text("mag_yaw"), options_group);
		debug_box = new QCheckBox(obs_module_text("debug_log"), options_group);
		// Resets every dock setting; lives at the bottom of the options
		// group, away from the everyday tracker buttons.
		auto *reset_defaults =
			new QPushButton(obs_module_text("reset_defaults"), options_group);
		options_layout->addWidget(mag_yaw_box);
		options_layout->addWidget(debug_box);
		options_layout->addWidget(reset_defaults);
		root->addWidget(options_group);
		root->addStretch();
		setWidget(content);

		QObject::connect(connect_box, &QCheckBox::toggled, this,
				 [](bool checked) { manager_set_connect_enabled(&g_device, checked); });
		QObject::connect(brightness_spin,
				 static_cast<void (QDoubleSpinBox::*)(double)>(
					 &QDoubleSpinBox::valueChanged),
				 this, [](double value) {
					 g_device.brightness_request.store(
						 static_cast<int>(
							 std::lround(value)),
						 std::memory_order_relaxed);
				 });
		QObject::connect(autobright_box, &QCheckBox::toggled, this,
				 [](bool checked) {
					 g_device.autobright_request.store(
						 checked ? 1 : 0,
						 std::memory_order_relaxed);
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
		QObject::connect(auto_projector_box, &QCheckBox::toggled, this,
				 [](bool checked) {
					 g_device.auto_projector.store(
						 checked,
						 std::memory_order_relaxed);
				 });
		QObject::connect(auto_monitor_box, &QCheckBox::toggled, this,
				 [this](bool checked) {
					 g_device.auto_monitor.store(
						 checked,
						 std::memory_order_relaxed);
					 // Re-arm so enabling applies on the
					 // next poll without replugging.
					 if (checked)
						 auto_monitor_applied = false;
				 });
		// The fence itself rises/falls on the next poll tick, which
		// also knows the current glasses-display rect.
		QObject::connect(cursor_fence_box, &QCheckBox::toggled, this,
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
		QObject::connect(dist_marker_box, &QCheckBox::toggled, this,
				 [](bool checked) {
					 g_device.screen_dist_from_marker.store(
						 checked,
						 std::memory_order_relaxed);
				 });
		QObject::connect(fov_auto_box, &QCheckBox::toggled, this, [this](bool checked) {
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
		QObject::connect(mag_yaw_box, &QCheckBox::toggled, this,
				 [](bool checked) { manager_set_mag_yaw(&g_device, checked); });
		QObject::connect(debug_box, &QCheckBox::toggled, this, [](bool checked) {
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

		auto *slider = new QSlider(Qt::Horizontal, row);
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
					   ? obs_module_text("detected_device.none")
					   : QString::fromStdString(
						     profile_for(detected).name));
		// Transport-specific rows (currently the One-family TCP endpoint)
		// follow the detected device; nothing detected hides them all.
		const imu_transport transport = profile_for(detected).transport;
		transport_label->setText(
			obs_module_text(traits_for(transport).name_key));
		if (static_cast<int>(transport) != last_transport) {
			last_transport = static_cast<int>(transport);
			// No detected device means no IMU stream to enable; the
			// checkbox appears together with the device.
			device_form->setRowVisible(connect_box,
						   transport != imu_transport::none);
			const transport_traits tr = traits_for(transport);
			device_form->setRowVisible(ip_edit,
						   tr.uses_network_endpoint);
			device_form->setRowVisible(port_spin,
						   tr.uses_network_endpoint);
			device_form->setRowVisible(brightness_row,
						   tr.display_brightness);
			device_form->setRowVisible(autobright_box,
						   tr.display_brightness);
			// Display-mode choices follow the detected family.
			{
				QSignalBlocker block(display_mode_combo);
				display_mode_combo->clear();
				for (size_t i = 0; i < tr.display_mode_count; ++i)
					display_mode_combo->addItem(
						obs_module_text(
							tr.display_modes[i]
								.label_key),
						tr.display_modes[i].value);
			}
			device_form->setRowVisible(display_mode_combo,
						   tr.display_mode_count > 0);
			device_form->setRowVisible(eye_label, tr.eye_camera);
			device_form->setRowVisible(eye_button, tr.eye_camera);
			device_form->setRowVisible(dist_marker_box,
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
				eye_text = obs_module_text("dock.eye.switching");
			else if (eye_present < 0)
				eye_text = obs_module_text("dock.eye.unknown");
			else if (eye_present == 0)
				eye_text = obs_module_text("dock.eye.absent");
			else
				eye_text = obs_module_text(
					eye_uvc == 1 ? "dock.eye.uvc_on"
						     : "dock.eye.uvc_off");
			eye_label->setText(eye_text);
			eye_button->setText(obs_module_text(
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
		stream_label->setText(!enabled ? obs_module_text("dock.stream.disabled")
					       : (connected
							  ? obs_module_text("dock.stream.connected")
							  : obs_module_text("dock.stream.waiting")));
		const char *pose_status = obs_module_text("dock.pose.disconnected");
		if (!enabled) {
			pose_status = obs_module_text("dock.pose.disabled");
		} else if (connected && p.connected) {
			pose_status = p.calibrated
					      ? obs_module_text("dock.pose.calibrated")
					      : obs_module_text("dock.pose.calibrating");
		}
		pose_label->setText(pose_status);
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
		cursor_fence_update(
			g_device.cursor_fence.load(std::memory_order_relaxed),
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
				: QString(obs_module_text(
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
		{
			QSignalBlocker block(auto_monitor_box);
			auto_monitor_box->setChecked(g_device.auto_monitor.load(
				std::memory_order_relaxed));
		}
		// Monitoring auto-switch: one latch per detected connection.
		// USB audio can show up later than HID, so retry on every
		// poll until the endpoint exists.
		if (detected == MODEL_UNKNOWN) {
			auto_monitor_applied = false;
		} else if (g_device.auto_monitor.load(std::memory_order_relaxed) &&
			   !auto_monitor_applied &&
			   apply_glasses_monitoring_device()) {
			auto_monitor_applied = true;
		}
		{
			QSignalBlocker block(cursor_fence_box);
			cursor_fence_box->setChecked(g_device.cursor_fence.load(
				std::memory_order_relaxed));
		}
		if (!glasses_display_present) {
			// Windows moves a removed display's windows onto the
			// remaining monitors, which would leave the fullscreen
			// projector covering a desktop screen. Close the
			// projectors that were sitting on the glasses display
			// instead, and re-arm the auto-open for the next
			// connection.
			for (const QPointer<QWidget> &projector :
			     glasses_projectors) {
				if (projector)
					projector->close();
			}
			glasses_projectors.clear();
			auto_projector_opened = false;
		} else {
			if (g_device.auto_projector.load(
				    std::memory_order_relaxed) &&
			    !auto_projector_opened && detected != MODEL_UNKNOWN &&
			    virtual_count > 0) {
				if (open_glasses_source_projector(false))
					auto_projector_opened = true;
			}
			// Track every projector currently on the glasses
			// screen (manually opened ones included) so they can
			// be closed when the display disappears.
			glasses_projectors.clear();
			QScreen *glasses_screen = QGuiApplication::screens().value(
				projector_monitor_index(glasses));
			for (QWidget *projector :
			     projectors_on_screen(glasses_screen))
				glasses_projectors.append(projector);
		}

		{
			QSignalBlocker block(connect_box);
			connect_box->setChecked(enabled);
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
		{
			QSignalBlocker block(dist_marker_box);
			dist_marker_box->setChecked(
				g_device.screen_dist_from_marker.load(
					std::memory_order_relaxed));
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
			QSignalBlocker block(mag_yaw_box);
			mag_yaw_box->setChecked(g_device.mag_yaw.load(std::memory_order_relaxed));
		}
		{
			QSignalBlocker block(debug_box);
			debug_box->setChecked(g_device.debug_log.load(std::memory_order_relaxed));
		}

		const double diag_m =
			2.0 * DEFAULT_SCREEN_DISTANCE_M * std::tan(fov * PI / 360.0) *
			size_factor;
		const double diag_in = diag_m / 0.0254;
		const double apparent_fov = 2.0 * std::atan(diag_m / (2.0 * distance)) *
					    180.0 / PI;
		screen_label->setText(QString::asprintf("%.1f in / %.1f deg", diag_in,
							apparent_fov));
	}

	QLabel *hid_label = nullptr;
	QLabel *glasses_display_label = nullptr;
	QLabel *transport_label = nullptr;
	QLabel *stream_label = nullptr;
	QLabel *pose_label = nullptr;
	QLabel *virtual_label = nullptr;
	QLabel *screen_label = nullptr;
	QCheckBox *connect_box = nullptr;
	QFormLayout *device_form = nullptr;
	QLineEdit *ip_edit = nullptr;
	QSpinBox *port_spin = nullptr;
	QDoubleSpinBox *brightness_spin = nullptr;
	QSlider *brightness_slider = nullptr;
	QWidget *brightness_row = nullptr;
	QCheckBox *autobright_box = nullptr;
	QComboBox *display_mode_combo = nullptr;
	QLabel *eye_label = nullptr;
	QPushButton *eye_button = nullptr;
	QComboBox *sbs_combo = nullptr;
	int last_transport = -1; // imu_transport value last applied to row visibility
	QDoubleSpinBox *prediction_spin = nullptr;
	QSlider *prediction_slider = nullptr;
	QCheckBox *fov_auto_box = nullptr;
	QCheckBox *dist_marker_box = nullptr;
	QDoubleSpinBox *fov_spin = nullptr;
	QSlider *fov_slider = nullptr;
	QDoubleSpinBox *distance_spin = nullptr;
	QSlider *distance_slider = nullptr;
	QDoubleSpinBox *size_spin = nullptr;
	QSlider *size_slider = nullptr;
	QDoubleSpinBox *curve_spin = nullptr;
	QSlider *curve_slider = nullptr;
	QDoubleSpinBox *ipd_spin = nullptr;
	QSlider *ipd_slider = nullptr;
	QCheckBox *mag_yaw_box = nullptr;
	QCheckBox *debug_box = nullptr;
	QPushButton *projector_button = nullptr;
	QCheckBox *auto_projector_box = nullptr;
	QCheckBox *auto_monitor_box = nullptr;
	QCheckBox *cursor_fence_box = nullptr;
	// Auto-open latch: one projector per glasses-display connection,
	// re-armed when a virtual screen source first appears.
	bool auto_projector_opened = false;
	int last_virtual_count = -1;
	// Monitoring-device latch: one auto-switch per detected connection.
	bool auto_monitor_applied = false;
	// Projectors seen on the glasses screen; closed on display removal.
	QList<QPointer<QWidget>> glasses_projectors;
	QTimer *timer = nullptr;
};

void init_dock()
{
	auto *parent = reinterpret_cast<QWidget *>(obs_frontend_get_main_window());
	auto *widget = new NyanRealDock(parent);
	if (!obs_frontend_add_dock_by_id("nyan_real_3dof_dock",
					 obs_module_text("dock.title"), widget)) {
		delete widget;
		blog(LOG_ERROR,
		     "[obs-nyan-real-3dof] failed to register dock UI; check for duplicate plugin instances");
		return;
	}
	if (auto *dock = qobject_cast<QDockWidget *>(widget->parentWidget())) {
		dock->setAllowedAreas(Qt::AllDockWidgetAreas);
		dock->setFloating(false);
	} else {
		blog(LOG_WARNING,
		     "[obs-nyan-real-3dof] dock UI registered, but QDockWidget parent was not found");
	}
	blog(LOG_INFO,
	     "[obs-nyan-real-3dof] dock UI registered; enable it from the OBS Docks menu");
}

void shutdown_dock()
{
	cursor_fence_shutdown();
	obs_frontend_remove_dock("nyan_real_3dof_dock");
}
#else
void init_dock()
{
	blog(LOG_WARNING,
	     "[obs-nyan-real-3dof] Qt was not available at build time; dock UI disabled");
}

void shutdown_dock() {}
#endif
