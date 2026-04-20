#include "gui/context/states.hh"

using namespace creeper;
namespace pcs::gui::context {

AppStates::AppStates(creeper::ThemeManager& manager, AppModules& modules) noexcept {
    navigation    = std::make_unique<NavigationState>(manager);
    visualization = std::make_unique<VisualizationWindowState>(manager, *modules.renderer);
    working = std::make_unique<WorkingPanelState>(
        manager, *modules.assets, *modules.runtime, *modules.renderer);

    navigation->mouse               = modules.mouse.get();
    visualization->runtime          = modules.runtime.get();
    visualization->mouse            = modules.mouse.get();
    working->mouse                  = modules.mouse.get();
    working->open_control           = modules.open_control.get();
    working->action_panel_registry  = modules.action_panels.get();
    working->asset_details_registry = modules.asset_details.get();
}

}
