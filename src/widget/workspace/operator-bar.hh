#pragma once

#include <qobjectdefs.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <ui_operator.h>

#include "operators/coordinate.hh"
#include "operators/selector.hh"

namespace workspace {
class OperatorBar : public QWidget, Ui::OperateBar {
    Q_OBJECT
public:
    explicit OperatorBar(QWidget* parent = nullptr)
        : QWidget(parent) {
        setupUi(this);
        setFixedWidth(30);

        iconLayout->setAlignment(Qt::AlignTop);

        auto* selection = new SelectButton(":pic/selection.svg");
        auto* coordinate = new CoordinateButton(":pic/coordination.svg");

        iconLayout->addWidget(selection);
        iconLayout->addWidget(coordinate);
    }
};
}