#include "gui/context/modules.hh"

namespace pcs::gui::context {

AppModules::AppModules() noexcept {
    runtime       = std::make_unique<pcs::Runtime>();
    renderer      = std::make_unique<pcs::Renderer>();
    assets        = std::make_unique<pcs::AssetsManager>(*renderer);
    mouse         = std::make_unique<pcs::gui::interaction::Mouse>();
    action_panels = std::make_unique<pcs::gui::working::ActionPanelRegistry>();
    open_control  = std::make_unique<pcs::gui::working::OpenControl>();
    asset_details = std::make_unique<pcs::gui::working::AssetDetailsRegistry>();
}

}
