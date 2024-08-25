#pragma once

#include <qobjectdefs.h>
#include <qwidget.h>
#include <ui_operator.h>

class OperatorBar : public QWidget {
    Q_OBJECT
public:
    OperatorBar(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        page_.setupUi(this);
        setFixedWidth(30);
    }

private:
    Ui::Operator page_;
};