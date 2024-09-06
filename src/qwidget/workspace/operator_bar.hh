#pragma once

#include "operators.hh"

#include <qobjectdefs.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <ui_operator.h>

namespace workspace {
class OperatorBar : public QWidget, Ui::Operator {
    Q_OBJECT
public:
    OperatorBar(QWidget* parent = nullptr)
        : QWidget(parent) {
        setupUi(this);
        setFixedWidth(30);

        coordinate_.connectPushButton(operator1);
    }

private:
    Coordinate coordinate_;
    PointPicker pointPicker_;
};
}