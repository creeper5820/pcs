#pragma once

#include "core/assets.hh"
#include "core/runtime.hh"
#include "gui/interaction/mouse.hh"
#include "gui/working/asset-details.hh"
#include "gui/working/open-control.hh"

#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/utility/wrapper/mutable-value.hh>
#include <creeper-qt/utility/wrapper/pimpl.hh>

#include <QWidget>

namespace pcs::gui::working {
class ActionPanelRegistry;
}

class WorkingPanel final : public QWidget {
    CREEPER_PIMPL_DEFINITION(WorkingPanel)

public:
    WorkingPanel(creeper::ThemeManager& manager, pcs::AssetsManager& assets, pcs::Runtime& runtime,
        pcs::Renderer& renderer, pcs::gui::working::OpenControl& open_control,
        pcs::gui::working::AssetDetailsRegistry& asset_details_registry,
        pcs::gui::interaction::Mouse& mouse,
        pcs::gui::working::ActionPanelRegistry& action_panel_registry,
        creeper::MutableDouble& panel_width, bool& assets_visibility) noexcept;

    auto refresh_assets_list() noexcept -> void;
    auto select_asset(std::string const& id) noexcept -> void;
    auto save_current_asset() noexcept -> void;
};
