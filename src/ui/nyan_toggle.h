// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// A modern on/off switch, used as a drop-in for QCheckBox in the dock and the
// standalone control window. It is a checkable QAbstractButton (same
// setChecked/isChecked/toggled/text API), self-painted so it looks the same on
// any theme (colours follow QPalette). Header-only, no Q_OBJECT - it adds no new
// signals/slots and inherits `toggled` from QAbstractButton.
#pragma once

#include <QtCore/QSize>
#include <QtGui/QPainter>
#include <QtGui/QPaintEvent>
#include <QtGui/QPalette>
#include <QtWidgets/QAbstractButton>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QWidget>

class ToggleSwitch : public QAbstractButton {
public:
	explicit ToggleSwitch(const QString &text = QString(),
			      QWidget *parent = nullptr)
		: QAbstractButton(parent)
	{
		setCheckable(true);
		setText(text);
		setCursor(Qt::PointingHandCursor);
	}
	explicit ToggleSwitch(QWidget *parent) : ToggleSwitch(QString(), parent) {}

	QSize sizeHint() const override
	{
		int w = track_w_;
		if (!text().isEmpty())
			w += gap_ + fontMetrics().horizontalAdvance(text());
		return QSize(w, qMax(track_h_, fontMetrics().height()));
	}
	QSize minimumSizeHint() const override { return sizeHint(); }

protected:
	void paintEvent(QPaintEvent *) override
	{
		QPainter p(this);
		p.setRenderHint(QPainter::Antialiasing, true);
		const qreal cy = height() / 2.0;
		const bool on = isChecked();

		// Track.
		const QRectF track(0.5, cy - track_h_ / 2.0 + 0.5, track_w_ - 1.0,
				   track_h_ - 1.0);
		QColor track_col = on ? palette().color(QPalette::Highlight)
				      : palette().color(QPalette::Mid);
		if (!isEnabled())
			track_col.setAlpha(90);
		p.setPen(Qt::NoPen);
		p.setBrush(track_col);
		p.drawRoundedRect(track, track.height() / 2.0, track.height() / 2.0);

		// Knob.
		const qreal d = track_h_ - 6.0;
		const qreal x = on ? track_w_ - d - 3.0 : 3.0;
		p.setBrush(QColor(245, 245, 245));
		p.drawEllipse(QRectF(x, cy - d / 2.0, d, d));

		// Label (to the right of the switch), like QCheckBox text.
		if (!text().isEmpty()) {
			p.setPen(palette().color(isEnabled() ? QPalette::Active
							     : QPalette::Disabled,
						 QPalette::WindowText));
			p.drawText(QRectF(track_w_ + gap_, 0,
					  width() - track_w_ - gap_, height()),
				   Qt::AlignVCenter | Qt::AlignLeft, text());
		}
	}

private:
	int track_w_ = 40;
	int track_h_ = 22;
	int gap_ = 8;
};

// Add a "label on the left, switch on the right" form row: the label goes in the
// form's label column (so every row's label aligns), the (text-less) switch is
// right-aligned in the field column (its right edge lines up with the combos /
// fields above and below). Keeps a settings list tidy instead of scattering the
// toggle labels. Shared by the dock and the standalone control window.
//
// Returns the field wrapper widget: pass THIS (not the switch) to
// QFormLayout::setRowVisible() to show/hide the whole row, since the switch is
// nested inside the wrapper and the form keys rows by their field widget.
inline QWidget *add_switch_row(QFormLayout *form, const QString &label,
			       ToggleSwitch *sw)
{
	auto *wrap = new QWidget();
	auto *hl = new QHBoxLayout(wrap);
	hl->setContentsMargins(0, 0, 0, 0);
	hl->addStretch(1);
	hl->addWidget(sw);
	form->addRow(label, wrap);
	return wrap;
}
