// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// The nyan Real 3DoF dock widget (Qt, OBS-independent). The host shell creates
// it and hosts it (OBS frontend dock, or the standalone window). Defined only
// when built with Qt.
#pragma once

class QWidget;

// Builds the dock control widget bound to the shared engine. UI thread only.
QWidget *create_nyan_real_dock(QWidget *parent);
