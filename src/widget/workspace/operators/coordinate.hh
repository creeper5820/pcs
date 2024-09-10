#pragma once

#include "core/renderer/renderer.hh"
#include "widget/workspace/operators/button.hh"

namespace workspace {
class CoordinateButton : public OperateButton {
public:
    CoordinateButton(const QString& url)
        : OperateButton(url) {
    }

protected:
    void onLeftMouseClicked() override {
        static auto& renderer = Renderer::instance();
        static auto coordinate = renderer.makeCoordinate({}, 10, 0.2);
        static auto flag = false;
        coordinate->setVisible(flag = !flag);
        renderer.render();
    }
    void onRightMouseClicked() override {
    }
};
}