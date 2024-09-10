#pragma once

#include "ui_selector.h"
#include "widget/workspace/operators/button.hh"
#include <qwidget.h>

namespace workspace {
class SelectPanel : public QWidget, Ui::Selector {
public:
    SelectPanel(QWidget* parent = nullptr)
        : QWidget(parent) {
        Selector::setupUi(this);
    }

private:
};

class SelectButton : public OperateButton {
public:
    SelectButton(const QString& url)
        : OperateButton(url) {
    }

protected:
    void onLeftMouseClicked() override {
    }
    void onRightMouseClicked() override {
    }
};
}