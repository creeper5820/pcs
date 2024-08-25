#pragma once

#include <qobjectdefs.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <ui_operator.h>

#include "core/viewer/cloud.hpp"

class OperatorBar : public QWidget {
    Q_OBJECT
public:
    OperatorBar(QWidget* parent = nullptr)
        : QWidget(parent) {
        page_.setupUi(this);
        setFixedWidth(30);

        connect(page_.operator1, &QPushButton::clicked, this,
            [=]() { core::viewer::storage.set_coordinate_system(2.0, "frame"); });
    }

private:
    Ui::Operator page_;
};