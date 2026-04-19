#include "gui/context/modules.hh"

namespace pcs::gui::context {

auto make_app_modules() noexcept -> AppModules {
    auto modules          = AppModules { };
    modules.runtime       = std::make_unique<pcs::Runtime>();
    modules.renderer      = std::make_unique<pcs::Renderer>();
    modules.assets        = std::make_unique<pcs::AssetsManager>(*modules.renderer);
    modules.mouse         = std::make_unique<pcs::gui::interaction::Mouse>();
    modules.action_panels = std::make_unique<pcs::gui::working::ActionPanelRegistry>();
    modules.open_control  = std::make_unique<pcs::gui::working::OpenControl>();
    modules.asset_details = std::make_unique<pcs::gui::working::AssetDetailsRegistry>();
    return modules;
}

}
