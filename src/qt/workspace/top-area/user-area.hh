#pragma once

#include "utility/common.hh"
#include <creeper-qt/widget/widget.hh>

namespace qt {

class UserArea : public creeper::Extension<QWidget> {
    Q_OBJECT
    WIDGET_PIMPL_DEFINTION(UserArea)
};

}