#pragma once

#include "gui/context/modules.hh"
#include "gui/navigation.hh"
#include "gui/visualization-window.hh"
#include "gui/working-panel.hh"

#include <memory>

namespace pcs::gui::context {

struct AppStates {
    std::unique_ptr<NavigationState> navigation;
    std::unique_ptr<VisualizationWindowState> visualization;
    std::unique_ptr<WorkingPanelState> working;
};

auto make_app_states(creeper::ThemeManager& manager, AppModules& modules) noexcept -> AppStates;

}
