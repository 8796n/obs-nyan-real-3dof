// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
#include "dock_host_obs.h"

#include <obs-module.h>

#ifdef NYAN_REAL_3DOF_WITH_QT_DOCK

#include <obs-frontend-api.h>

#include <QApplication>
#include <QDockWidget>
#include <QGuiApplication>
#include <QPointer>
#include <QScreen>
#include <QWidget>

#include <cstring>
#include <string>

#include "cursor_fence.h"
#include "dock.h"
#include "nyan_host.h"
#include "virtual_source.h"

namespace {

// Glasses-screen projector windows snapshotted while the display is present, so
// they can be closed when it disappears (the screen index is gone by then).
QList<QPointer<QWidget>> g_tracked;

// First nyan Real virtual screen source, used as the projector content.
bool find_virtual_source_name(std::string &name_out)
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

// Projector windows are OBS's own QWidget top-levels of class OBSProjector.
QList<QWidget *> projectors_on_screen(QScreen *screen)
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

// Close existing projector windows on the target screen so the dock button and
// the auto-open never stack projectors. obs_frontend_open_projector opens a new
// window on every call, and OBS only deduplicates when the user enabled "Limit
// one fullscreen projector per screen".
void close_projectors_on_screen(QScreen *screen)
{
	for (QWidget *widget : projectors_on_screen(screen))
		widget->close();
}

// Opens the virtual screen's fullscreen source projector on the given Qt screen
// index (resolved by the shared dock from the glasses display). UI thread only.
bool open_glasses_output(int monitor, bool log)
{
	std::string source_name;
	if (!find_virtual_source_name(source_name)) {
		if (log)
			blog(LOG_WARNING,
			     "[obs-nyan-real-3dof] no virtual screen source exists; add one before opening the projector");
		return false;
	}
	close_projectors_on_screen(QGuiApplication::screens().value(monitor));
	obs_frontend_open_projector("Source", monitor, nullptr,
				    source_name.c_str());
	if (log)
		blog(LOG_INFO,
		     "[obs-nyan-real-3dof] opened source projector '%s' (monitor %d)",
		     source_name.c_str(), monitor);
	return true;
}

// Snapshot every projector currently on the glasses screen (manually opened
// ones included) so close_glasses_output can tear them all down.
void track_glasses_output(int monitor)
{
	g_tracked.clear();
	QScreen *screen = QGuiApplication::screens().value(monitor);
	for (QWidget *projector : projectors_on_screen(screen))
		g_tracked.append(projector);
}

// Windows moves a removed display's windows onto the remaining monitors, which
// would leave the fullscreen projector covering a desktop screen. Close the
// projectors that were sitting on the glasses display instead.
void close_glasses_output()
{
	for (const QPointer<QWidget> &projector : g_tracked) {
		if (projector)
			projector->close();
	}
	g_tracked.clear();
}

} // namespace

void register_dock_host_obs()
{
	nyan_set_glasses_output_opener(open_glasses_output);
	nyan_set_glasses_output_tracker(track_glasses_output);
	nyan_set_glasses_output_closer(close_glasses_output);
}

void init_dock()
{
	auto *parent = reinterpret_cast<QWidget *>(obs_frontend_get_main_window());
	QWidget *widget = create_nyan_real_dock(parent);
	if (!obs_frontend_add_dock_by_id("nyan_real_3dof_dock",
					 nyan_text("dock.title"), widget)) {
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

void register_dock_host_obs() {}

void init_dock()
{
	blog(LOG_WARNING,
	     "[obs-nyan-real-3dof] Qt was not available at build time; dock UI disabled");
}

void shutdown_dock() {}

#endif
