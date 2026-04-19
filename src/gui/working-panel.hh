#pragma once
#include "core/assets.hh"
#include "core/runtime.hh"
#include "gui/interaction/mouse.hh"
#include "gui/working/asset-details.hh"
#include "gui/working/open-control.hh"

#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/utility/wrapper/mutable-value.hh>
#include <creeper-qt/utility/wrapper/widget.hh>

#include <qpointer.h>
#include <qwidget.h>

namespace pcs::gui::working {
class ActionPanelRegistry;
}

struct WorkingPanelState {
    creeper::ThemeManager& manager;
    pcs::AssetsManager& assets;
    pcs::Runtime& runtime;
    pcs::Renderer& renderer;
    pcs::gui::working::OpenControl* open_control                    = nullptr;
    pcs::gui::working::AssetDetailsRegistry* asset_details_registry = nullptr;
    pcs::gui::interaction::Mouse* mouse                             = nullptr;
    pcs::gui::working::ActionPanelRegistry* action_panel_registry   = nullptr;

    creeper::MutableDouble panel_width { 300. };

    bool assets_visibility = true;
};
auto WorkingPanelComponent(WorkingPanelState&) noexcept -> QPointer<QWidget>;
