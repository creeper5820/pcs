#include "gui/context/states.hh"

namespace pcs::gui::context {

auto make_app_states(creeper::ThemeManager& manager, AppModules& modules) noexcept -> AppStates {
    auto states          = AppStates { };
    states.navigation    = std::make_unique<NavigationState>(manager);
    states.visualization =
        std::make_unique<VisualizationWindowState>(manager, *modules.renderer);
    states.working = std::make_unique<WorkingPanelState>(
        manager, *modules.assets, *modules.runtime, *modules.renderer);

    states.navigation->mouse = modules.mouse.get();
    states.visualization->mouse = modules.mouse.get();
    states.working->mouse = modules.mouse.get();
    states.working->open_control = modules.open_control.get();
    states.working->action_panel_registry = modules.action_panels.get();
    states.working->asset_details_registry = modules.asset_details.get();

    return states;
}

}
