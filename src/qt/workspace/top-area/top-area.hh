#pragma once

#include "utility/common.hh"
#include <creeper-qt/widget/basic-shape.hh>

namespace qt {

class TopArea : public creeper::Rectangle {
    WIDGET_PIMPL_DEFINTION(TopArea);
    Q_OBJECT

public:
    void setFileName(const QString& name);
};

}