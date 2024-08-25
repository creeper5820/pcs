#pragma once

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

#include "operator/operator.hpp"
#include "tool_bar/tool_bar.hpp"
#include "utility/common.hpp"
#include "viewer/viewer.hpp"

namespace workspace {

class Workspace final : public QMainWindow {
    Q_OBJECT
public:
    explicit Workspace() {
        page_.setupUi(this);

        page_.button_exit->setStyleSheet(
            utility::style(":qss/button/exit.qss"));
        page_.button_layout->setStyleSheet(
            utility::style(":qss/button/layout.qss"));
        page_.button_about->setStyleSheet(
            utility::style(":qss/button/about.qss"));
        page_.menu->setStyleSheet(
            utility::style(":qss/bar/menu.qss"));
        page_.statusBar->setStyleSheet(
            utility::style(":qss/bar/status.qss"));

        // layout
        auto& layout = page_.layout_workspace;
        layout->addWidget(&operator_);
        layout->setStretch(1, 0);
        layout->addWidget(&viewer_);
        layout->setStretch(2, 1);
        layout->addWidget(&tool_);
        layout->setStretch(3, 0);

        connect(page_.button_exit, &QPushButton::clicked,
            this, &Workspace::exit);
        connect(page_.button_layout, &QPushButton::clicked,
            this, &Workspace::toggle_layout);
    }

private slots:
    void exit() {
        auto answer = QMessageBox::information(
            this, "info", "Exit?", QMessageBox::Ok | QMessageBox::No);
        if (answer == QMessageBox::Ok)
            QCoreApplication::exit(0);
    }
    void toggle_layout() {
        static auto toggle { true };
        static auto current_size = size();
        static auto current_pos = pos();

        constexpr auto k = 0.95;

        auto screen = QGuiApplication::primaryScreen()->size();
        auto center = QPoint(
            (1. - k) / 2. * screen.width(), (1. - k) / 2. * screen.height());

        utility::message(center.x(), center.y());

        if (toggle) {
            current_size = size();
            current_pos = pos();
            setFixedSize(screen * k);
            move(center);
        } else {
            setFixedSize(current_size);
            move(current_pos);
        }

        toggle = !toggle;
    }

private:
    Ui::Workspace page_;
    ToolBar tool_;
    Viewer viewer_;
    OperatorBar operator_;
};

} // namespace workspace