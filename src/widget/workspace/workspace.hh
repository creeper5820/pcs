#pragma once

#include "core/operator/interactor/point-picker.hh"
#include "utility/common.hh"
#include "widget/workspace/cloud-editor.hh"
#include "widget/workspace/operator-bar.hh"
#include "widget/workspace/view.hh"

#include "qdatetime.h"
#include <qaction.h>
#include <qcoreapplication.h>
#include <qmainwindow.h>
#include <qmessagebox.h>
#include <qnamespace.h>
#include <qobjectdefs.h>
#include <qpainter.h>
#include <qpoint.h>
#include <qpushbutton.h>
#include <qscreen.h>
#include <qstyle.h>
#include <qwidget.h>
#include <ui_workspace.h>

namespace workspace {

/// @brief workspace as you can see, nothing more
class Workspace final : public QMainWindow, public Ui::Workspace {
    Q_OBJECT
public:
    explicit Workspace() {
        setupUi(this);

        // load qss style
        button_exit->setStyleSheet(
            util::style(":qss/button/exit.qss"));
        button_layout->setStyleSheet(
            util::style(":qss/button/layout.qss"));
        button_about->setStyleSheet(
            util::style(":qss/button/about.qss"));
        menu->setStyleSheet(
            util::style(":qss/bar/menu.qss"));
        messageBar->setStyleSheet(
            util::style(":qss/bar/status.qss"));

        // layout
        auto& layout = workspaceLayout;
        layout->addWidget(&operator_);
        layout->setStretch(1, 0);
        layout->addWidget(&view_);
        layout->setStretch(2, 1);
        layout->addWidget(&tool_);
        layout->setStretch(3, 0);

        // connect button with callback
        connect(button_exit, &QPushButton::clicked,
            this, &Workspace::onExitButtonClicked);
        connect(button_layout, &QPushButton::clicked,
            this, &Workspace::onToggleLayoutButtonClicked);

        auto& point = core::operators::clickedPoint;
        auto& isClicked = core::operators::isClicked;
        view_.registerOnMouseRelease([this](QMouseEvent* event) {
            if (isClicked) {
                auto clickMessage = QString("Clicked: (")
                    + QString::number(point.x(), 'f', 3) + ", "
                    + QString::number(point.y(), 'f', 3) + ", "
                    + QString::number(point.z(), 'f', 3) + ")";
                showStatusWithTime(clickMessage);
                isClicked = false;
            }
        });

        // Title bar buttons
        auto* menu = new QMenu();
        menu->addAction("NormalStyle", [this] {
            Operators::instance().useNormalStyle();
        });
        menu->addAction("PointPicker", [this] {
            Operators::instance().usePointPicker();
        });
        button_settings->setMenu(menu);
    }

private slots:
    void onExitButtonClicked() {
        auto answer = QMessageBox::information(
            this, "info", "Exit?", QMessageBox::Ok | QMessageBox::No);
        if (answer == QMessageBox::Ok)
            QCoreApplication::exit(0);
    }

    void onToggleLayoutButtonClicked() {
        constexpr auto k = 0.95;
        static auto toggle { false };
        static auto current_size = size();
        static auto current_pos = pos();

        const auto screen = QGuiApplication::primaryScreen()->size();
        const auto center = QPoint {
            static_cast<int>((1. - k) / 2. * screen.width()),
            static_cast<int>((1. - k) / 2. * screen.height())
        };

        if ((toggle = !toggle)) {
            current_size = size();
            current_pos = pos();
            setFixedSize(screen * k);
            move(center);
        } else {
            setFixedSize(current_size);
            move(current_pos);
        };
    }

    void showStatusWithTime(const QString& message) {
        auto timeMessage = QTime::currentTime().toString("[ hh:mm:ss ]");
        messageBar->showMessage(timeMessage + " " + message);
    }

private:
    OperatorBar operator_ { this };
    CloudEditor tool_ { this };
    View view_ { this };
};

} // namespace workspace