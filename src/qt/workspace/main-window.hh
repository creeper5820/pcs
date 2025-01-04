#pragma once

#include <creeper-qt/widget/main-window.hh>

#include "utility/common.hh"

namespace qt {

class Workspace : public creeper::MainWindow {
    PIMPL_DEFINTION(Workspace);
    Q_OBJECT
};

}