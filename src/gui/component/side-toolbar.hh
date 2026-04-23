#pragma once

#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/utility/wrapper/pimpl.hh>
#include <qwidget.h>

namespace gui {

class SideToolbar final {
    CREEPER_PIMPL_DEFINITION(SideToolbar);

public:
    explicit SideToolbar(creeper::ThemeManager& manager);

    auto gui() -> QWidget* const;
};

}
