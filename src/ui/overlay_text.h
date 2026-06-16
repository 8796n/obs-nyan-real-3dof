// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Rasterize a short status string into an RGBA bitmap for the in-glasses warp
// overlay (e.g. "Calibrating sensors…"). Qt-based (QImage/QPainter) so it is
// localized and crisp; both backends upload the result to their own texture
// (OBS gs_texture / standalone D3D11) and composite it centered. Called only
// when the message changes, so the per-frame path stays free of Qt.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct overlay_bitmap {
	int width = 0;
	int height = 0;
	std::vector<uint8_t> rgba; // width*height*4, straight (non-premultiplied)
};

// White centered text on a translucent rounded backdrop, sized to fit. Empty
// text (or a tiny font) yields an empty bitmap. Safe off the GUI thread (it
// paints into its own QImage).
overlay_bitmap overlay_text_rasterize(const std::string &utf8, int font_px);
