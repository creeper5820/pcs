#pragma once

#include <QMouseEvent>
#include <QPushButton>

#include "utility/common.hh"

namespace workspace {
class OperateButton : public QPushButton {
public:
    explicit OperateButton(const QString& url)
        : QPushButton() {
        QPushButton::setIcon(QPixmap(url));
        QPushButton::setStyleSheet(util::style(
            ":/qss/button/operate.qss"));
    }

protected:
    void mousePressEvent(QMouseEvent* event) override {
        if (Qt::RightButton == event->button())
            onRightMouseClicked();
        else
            onLeftMouseClicked();
    }

    virtual void onLeftMouseClicked() { }
    virtual void onRightMouseClicked() { }
};
}