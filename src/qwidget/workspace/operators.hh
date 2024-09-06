#pragma once

#include <qobjectdefs.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <ui_operator.h>

#include "core/renderer/renderer.hh"

namespace workspace {
class Operator : public QWidget {
    Q_OBJECT
public:
    void connectPushButton(QPushButton* button) {
        setContextMenuPolicy(Qt::CustomContextMenu);
        button->setIcon(makeIconPixmap());

        connect(button, &QWidget::customContextMenuRequested,
            this, &Operator::showCustomMenu);
        connect(button, &QPushButton::clicked,
            this, &Operator::onLeftMouseClick);
    }

private:
    Q_SLOT virtual void onLeftMouseClick() = 0;
    Q_SLOT virtual void showCustomMenu() = 0;
    virtual QIcon makeIconPixmap() = 0;
};

class Coordinate : public Operator {
private:
    void onLeftMouseClick() override {
        auto& renderer = core::Renderer::instance();
        static auto flag = false;
        static auto index = renderer.addCoordinateSystem({}, 10, 0.2);

        if ((flag = !flag)) {
            renderer.setStereoPropsVisible(index, true);
        } else {
            renderer.setStereoPropsVisible(index, false);
        }
    }

    void showCustomMenu() override {
    }

    QIcon makeIconPixmap() override {
        return QPixmap(":pic/coordination.svg");
    }
};

class PointPicker : public Operator {
private:
    void onLeftMouseClick() override {
    }

    void showCustomMenu() override {
    }

    QIcon makeIconPixmap() override {
        return QPixmap(":pic/coordination.svg");
    }
};
}