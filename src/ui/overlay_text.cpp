// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "overlay_text.h"

#include <cstring>

#include <QColor>
#include <QFont>
#include <QFontMetrics>
#include <QImage>
#include <QPainter>
#include <QRect>
#include <QString>

overlay_bitmap overlay_text_rasterize(const std::string &utf8, int font_px)
{
	overlay_bitmap out;
	if (utf8.empty() || font_px < 4)
		return out;

	const QString text = QString::fromUtf8(utf8.c_str());
	QFont font;
	font.setPixelSize(font_px);
	font.setBold(true);
	const QFontMetrics fm(font);
	const QRect tb = fm.boundingRect(text);
	const int padx = font_px;
	const int pady = font_px * 2 / 3;
	const int w = tb.width() + padx * 2;
	const int h = fm.height() + pady * 2;
	if (w <= 0 || h <= 0)
		return out;

	QImage img(w, h, QImage::Format_RGBA8888);
	img.fill(Qt::transparent);
	{
		QPainter p(&img);
		p.setRenderHint(QPainter::Antialiasing, true);
		p.setRenderHint(QPainter::TextAntialiasing, true);
		p.setPen(Qt::NoPen);
		p.setBrush(QColor(0, 0, 0, 160)); // translucent backdrop
		p.drawRoundedRect(0, 0, w, h, pady, pady);
		p.setFont(font);
		p.setPen(QColor(255, 255, 255, 235));
		p.drawText(QRect(0, 0, w, h), Qt::AlignCenter, text);
	}

	out.width = w;
	out.height = h;
	out.rgba.resize(static_cast<size_t>(w) * static_cast<size_t>(h) * 4);
	for (int y = 0; y < h; ++y)
		std::memcpy(out.rgba.data() + static_cast<size_t>(y) * w * 4,
			    img.constScanLine(y), static_cast<size_t>(w) * 4);
	return out;
}
