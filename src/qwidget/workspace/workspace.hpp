#pragma once

#include <qcoreapplication.h>
#include <qmainwindow.h>
#include <qmessagebox.h>
#include <qobjectdefs.h>
#include <qpainter.h>
#include <qpushbutton.h>
#include <qstyle.h>
#include <qwidget.h>
#include <ui_workspace.h>

#include "tool_bar/tool_bar.hpp"
#include "utility/common.hpp"
#include "viewer/viewer.hpp"

namespace workspace {

class Workspace final : public QMainWindow {
    Q_OBJECT
public:
    explicit Workspace()
    {
        page_.setupUi(this);

        page_.button_exit->setStyleSheet(
            utility::style(":qss/button/exit.qss"));
        page_.button_layout->setStyleSheet(
            utility::style(":qss/button/layout.qss"));
        page_.button_about->setStyleSheet(
            utility::style(":qss/button/about.qss"));
        page_.menu->setStyleSheet(
            utility::style(":qss/bar/menu.qss"));

        // layout
        auto& layout = page_.layout_workspace;
        layout->addWidget(&tool_bar_);
        layout->setStretch(1, 0);
        layout->addWidget(&viewer_);
        layout->setStretch(2, 1);

        connect(page_.button_exit, &QPushButton::clicked,
            this, &Workspace::exit);
    }
    ~Workspace() = default;

private slots:
    void exit()
    {
        auto answer = QMessageBox::information(this, "info",
            "Exit?", QMessageBox::Ok | QMessageBox::No);
        if (answer == QMessageBox::Ok)
            QCoreApplication::exit(0);
    }

private:
    Ui::Workspace page_;
    ToolBar tool_bar_;
    Viewer viewer_;
};

}