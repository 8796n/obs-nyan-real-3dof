// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Marker-6DoF tracker thread: captures the XREAL Eye tracking camera (UVC1)
// and feeds AprilTag-derived head positions into the global tracker.
#pragma once

struct device_manager;

void marker_worker_fn(device_manager *f);
