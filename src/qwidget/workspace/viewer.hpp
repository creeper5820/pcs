#pragma once

#include "core/viewer/cloud.hpp"
#include "utility/common.hpp"

#include <qmessagebox.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <ui_viewer.h>

namespace workspace {

class View : public QWidget, Ui::WorkspaceViewer {
    Q_OBJECT
public:
    View(QWidget* parent = nullptr)
        : QWidget(parent) {

        setupUi(this);

        frame->setStyleSheet(util::style(":qss/normal/viewer.qss"));

        core::View::instance().bind_viewer(vtkWidget);
    }

    ~View() { }

private:
};

} // namespace workspace