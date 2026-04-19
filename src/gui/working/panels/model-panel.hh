#pragma once

#include "gui/working/action-panels.hh"

#include <memory>

namespace pcs::gui::working {

auto make_model_panel(ActionPanelContext context, QFont const& font)
    -> std::unique_ptr<AssetActionPanel>;

}
