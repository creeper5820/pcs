#pragma once

#include "core/cloud/cloud.hh"
#include "utility/common.hh"

#include <qmessagebox.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <ui_view.h>

namespace workspace {
class View : public QWidget, Ui::WorkspaceViewer {
    Q_OBJECT
public:
    View(QWidget* parent = nullptr)
        : QWidget(parent) {

        setupUi(this);

        window_ = new QVTKOpenGLNativeWidget {};
        core::Cloud::instance().connectUI(window_);

        frame->setStyleSheet(util::style(":qss/normal/viewer.qss"));
        frame->layout()->addWidget(window_);
    }

    ~View() { }

private:
    QVTKOpenGLNativeWidget* window_;
};

} // namespace workspace