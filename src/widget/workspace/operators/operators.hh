#pragma once

#include <qobjectdefs.h>
#include <qpushbutton.h>
#include <qwidget.h>

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

        button_ = button;
    }

private:
    QPushButton* button_;

    Q_SLOT virtual void onLeftMouseClick() = 0;
    Q_SLOT virtual void showCustomMenu() = 0;
    virtual QIcon makeIconPixmap() = 0;
};
}