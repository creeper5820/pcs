#pragma once

#include "gui/working/action-panels.hh"

#include <memory>

namespace pcs::gui::working {

auto make_png_map_panel(ActionPanelContext context, QFont const& font)
    -> std::unique_ptr<AssetActionPanel>;

}
