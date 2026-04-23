#pragma once

#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/utility/wrapper/pimpl.hh>
#include <qpointer.h>
#include <qwidget.h>

namespace pcs {

class VtkWindow {
    CREEPER_PIMPL_DEFINITION(VtkWindow);

public:
    explicit VtkWindow(creeper::ThemeManager& manager);

    auto component() -> QWidget* const;
};

}
