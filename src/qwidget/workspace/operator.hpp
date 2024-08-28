#pragma once

#include <qobjectdefs.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <ui_operator.h>

#include "core/viewer/cloud.hpp"

class Operator : public QWidget {
    Q_OBJECT
public:
    explicit Operator() = default;

    void bind(QPushButton* button) {
        setContextMenuPolicy(Qt::CustomContextMenu);
        connect(button, &QWidget::customContextMenuRequested,
            this, &Operator::custom_menu);
        connect(button, &QPushButton::clicked,
            this, &Operator::click_callback);
        button->setIcon(make_icon());
    }

private:
    Q_SLOT virtual void click_callback() = 0;
    Q_SLOT virtual void custom_menu() = 0;
    virtual QIcon make_icon() = 0;
};

class Coordinate : public Operator {
    Q_OBJECT
private:
    Q_SLOT void click_callback() override {
        static auto flag { true };

        if (flag)
            core::view::instance.add_coordinate_system(2.0, "default");
        else
            core::view::instance.remove_coordinate_system("default");

        flag = !flag;
    }
    Q_SLOT void custom_menu() override {
    }
    QIcon make_icon() override {
        return QPixmap(":pic/coordination.svg");
    }
};

class OperatorBar : public QWidget, Ui::Operator {
    Q_OBJECT
public:
    OperatorBar(QWidget* parent = nullptr)
        : QWidget(parent) {
        setupUi(this);
        setFixedWidth(30);

        coordinate_.bind(operator1);
    }

private:
    Coordinate coordinate_;
};