// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Collapsible section widget shared by the dock and the standalone control
// window so both render the same header band + foldable body. Header-only,
// no Q_OBJECT (std::function callbacks instead of signals), so it drops into
// any Qt widget tree without moc.
#pragma once

#include <algorithm>
#include <functional>

#include <QtGui/QFont>
#include <QtGui/QMouseEvent>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

// Full-width clickable band for a section header. Plain QFrame so the QSS
// background applies; the click callback avoids Q_OBJECT/moc.
class DockSectionHeader final : public QFrame {
public:
	std::function<void()> on_click;

protected:
	void mousePressEvent(QMouseEvent *event) override
	{
		if (event->button() == Qt::LeftButton && on_click)
			on_click();
		QFrame::mousePressEvent(event);
	}
};

// Collapsible dock section: a full-width header band (arrow + bold title +
// optional collapsed-state summary) above a body widget. The gray-overlay
// band reads as a section divider on dark and light themes alike.
class DockSection final : public QWidget {
public:
	std::function<void(bool open)> on_toggled; // user clicks only

	DockSection(const QString &title, QWidget *body, QWidget *parent = nullptr)
		: QWidget(parent),
		  body_(body)
	{
		auto *lay = new QVBoxLayout(this);
		lay->setContentsMargins(0, 0, 0, 0);
		lay->setSpacing(2);
		header_ = new DockSectionHeader();
		header_->setObjectName("nyanDockSectionHeader");
		header_->setStyleSheet(
			"#nyanDockSectionHeader {"
			" background-color: rgba(128,128,128,0.16);"
			" border-radius: 4px; }"
			"#nyanDockSectionHeader:hover {"
			" background-color: rgba(128,128,128,0.28); }");
		header_->setCursor(Qt::PointingHandCursor);
		auto *hl = new QHBoxLayout(header_);
		hl->setContentsMargins(8, 4, 8, 4);
		hl->setSpacing(6);
		arrow_ = new QLabel(header_);
		title_ = new QLabel(title, header_);
		QFont title_font = title_->font();
		title_font.setBold(true);
		title_->setFont(title_font);
		summary_ = new QLabel(header_);
		summary_->setVisible(false);
		// Arrow pinned to the left edge; the title (+ collapsed
		// summary) sits in the true center of the band thanks to a
		// phantom spacer of the arrow's width on the right.
		arrow_->setText(QStringLiteral("▾"));
		int arrow_w = arrow_->sizeHint().width();
		arrow_->setText(QStringLiteral("▸"));
		arrow_w = std::max(arrow_w, arrow_->sizeHint().width());
		arrow_->setFixedWidth(arrow_w);
		auto *balance = new QLabel(header_);
		balance->setFixedWidth(arrow_w);
		hl->addWidget(arrow_);
		hl->addStretch(1);
		hl->addWidget(title_);
		hl->addWidget(summary_);
		hl->addStretch(1);
		hl->addWidget(balance);
		lay->addWidget(header_);
		lay->addWidget(body_);
		header_->on_click = [this]() { set_open(!open_, true); };
		set_open(true, false);
	}

	// Programmatic open/close (settings load): no callback, so the
	// persistence binding does not echo the value back.
	void set_expanded(bool open) { set_open(open, false); }

	void set_summary(const QString &text, const QString &tooltip)
	{
		summary_->setText(text);
		summary_->setToolTip(tooltip);
		summary_->setVisible(!open_ && !text.isEmpty());
	}

private:
	void set_open(bool open, bool notify)
	{
		open_ = open;
		arrow_->setText(open ? QStringLiteral("▾")
				     : QStringLiteral("▸"));
		body_->setVisible(open);
		summary_->setVisible(!open && !summary_->text().isEmpty());
		if (notify && on_toggled)
			on_toggled(open);
	}

	DockSectionHeader *header_ = nullptr;
	QLabel *arrow_ = nullptr;
	QLabel *title_ = nullptr;
	QLabel *summary_ = nullptr;
	QWidget *body_ = nullptr;
	bool open_ = true;
};
