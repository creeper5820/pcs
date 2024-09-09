#pragma once

#include <qmessagebox.h>
#include <qpushbutton.h>
#include <qstatusbar.h>
#include <qwidget.h>
#include <ui_view.h>

#include "core/operator/operator.hh"
#include "utility/common.hh"

#include <utility>

namespace workspace {
class View : public QWidget, public Ui::WorkspaceViewer {
    Q_OBJECT
public:
    explicit View(QWidget* parent = nullptr)
        : QWidget(parent) {
        setupUi(this);

        window_ = new QVTKOpenGLNativeWidget {};
        auto& operators = Operators::instance();

        operators.connectWidget(window_);

        frame->setStyleSheet(util::style(":qss/normal/viewer.qss"));
        frame->layout()->addWidget(window_);

        onMousePress = [](QMouseEvent*) {};
        onMouseRelease = [](QMouseEvent*) {};
    }

    void registerOnMousePress(std::function<void(QMouseEvent*)> callback) {
        onMousePress = std::move(callback);
    }
    void registerOnMouseRelease(std::function<void(QMouseEvent*)> callback) {
        onMouseRelease = std::move(callback);
    }

protected:
    void mousePressEvent(QMouseEvent* event) override {
        onMousePress(event);
        QWidget::mousePressEvent(event);
    }
    void mouseReleaseEvent(QMouseEvent* event) override {
        onMouseRelease(event);
        QWidget::mouseReleaseEvent(event);
    }

private:
    QVTKOpenGLNativeWidget* window_;

    std::function<void(QMouseEvent*)> onMousePress;
    std::function<void(QMouseEvent*)> onMouseRelease;
};

} // namespace workspace