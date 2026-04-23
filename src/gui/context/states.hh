#pragma once

#include <creeper-qt/utility/wrapper/mutable-value.hh>

namespace pcs::gui::context {

struct AppStates {
    creeper::MutableDouble working_panel_width { 300. };
    bool assets_visibility = true;
};

}
