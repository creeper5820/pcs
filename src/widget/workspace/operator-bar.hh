#pragma once

#include <qobjectdefs.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <ui_operator.h>

#include "operators/button.hh"
#include "operators/coordinate.hh"
#include "operators/selector.hh"

namespace workspace {

class TestButton : public OperateButton {
public:
    explicit TestButton(const QString& url)
        : OperateButton(url) {
    }

protected:
    void onLeftMouseClicked() override {
    }

    void onRightMouseClicked() override {
    }
};

class OperatorBar : public QWidget, Ui::Operator {
    Q_OBJECT
public:
    explicit OperatorBar(QWidget* parent = nullptr)
        : QWidget(parent) {
        setupUi(this);
        setFixedWidth(30);

        operateLayout->setAlignment(Qt::AlignTop);

        auto* button0 = new TestButton(":pic/selection.svg");
        auto* button1 = new TestButton(":pic/selector.svg");
        auto* button2 = new TestButton(":pic/clear.svg");

        operateLayout->addWidget(button0);
        operateLayout->addWidget(button1);
        operateLayout->addWidget(button2);
    }

private:
    Coordinate coordinate_;
    SelectPanel selector_;
};
}