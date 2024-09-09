#pragma once

#include "core/renderer/renderer.hh"
#include "operators.hh"

namespace workspace {
class Coordinate : public Operator {
private:
    void onLeftMouseClick() override {
        auto& renderer = Renderer::instance();
        static auto flag = false;
        static auto coordinate = renderer.makeCoordinate({}, 10, 0.2);
        coordinate->setVisible(flag = !flag);
        renderer.render();
    }

    void showCustomMenu() override {
    }

    QIcon makeIconPixmap() override {
        return QPixmap(":pic/coordination.svg");
    }
};
}