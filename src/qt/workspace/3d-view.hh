#pragma once

#include <creeper-qt/widget/basic-shape.hh>

#include "utility/common.hh"

namespace qt {

class ThreeDView : public creeper::RoundedRectangle {
    WIDGET_PIMPL_DEFINTION(ThreeDView);
    Q_OBJECT

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
};

}