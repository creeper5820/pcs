#pragma once

#include "operators.hh"
#include "ui_operator.h"

#include <qmenu.h>

namespace workspace {
class SelectPanel : public Operator, Ui::Operator {
private:
    void onLeftMouseClick() override {
    }

    void showCustomMenu() override {
        auto menu = QMenu {};
        menu.addAction("delete", [] {});
        menu.exec(QCursor::pos());
    }

    QIcon makeIconPixmap() override {
        return QPixmap(":pic/selection.svg");
    }
};
}