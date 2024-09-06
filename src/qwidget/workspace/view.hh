#pragma once

#include <qmessagebox.h>
#include <qpushbutton.h>
#include <qstatusbar.h>
#include <qwidget.h>
#include <ui_view.h>

#include "core/operator/operator.hh"
#include "utility/common.hh"

namespace workspace {
class View : public QWidget, public Ui::WorkspaceViewer {
    Q_OBJECT
public:
    View(QWidget* parent = nullptr)
        : QWidget(parent) {
        setupUi(this);

        window_ = new QVTKOpenGLNativeWidget {};
        auto& operators = core::Operators::instance();

        operators.connectWidget(window_);

        frame->setStyleSheet(util::style(":qss/normal/viewer.qss"));
        frame->layout()->addWidget(window_);

        onMousePress = [](QMouseEvent*) { };
        onMouseRelease = [](QMouseEvent*) { };
    }

    void registerOnMousePress(std::function<void(QMouseEvent*)> callback) {
        onMousePress = callback;
    }
    void registerOnMouseRelease(std::function<void(QMouseEvent*)> callback) {
        onMouseRelease = callback;
    }

protected:
    void mousePressEvent(QMouseEvent* event) override { onMousePress(event); }
    void mouseReleaseEvent(QMouseEvent* event) override { onMouseRelease(event); }

private:
    QVTKOpenGLNativeWidget* window_;

    std::function<void(QMouseEvent*)> onMousePress;
    std::function<void(QMouseEvent*)> onMouseRelease;
};

} // namespace workspace