#pragma once

#include "core/assets.hh"
#include "core/renderer.hh"
#include "core/runtime.hh"
#include "gui/interaction/mouse.hh"
#include "gui/working/action-panels.hh"
#include "gui/working/asset-details.hh"
#include "gui/working/open-control.hh"

#include <memory>

namespace pcs::gui::context {

struct AppModules {
    std::unique_ptr<pcs::Runtime> runtime;
    std::unique_ptr<pcs::Renderer> renderer;
    std::unique_ptr<pcs::AssetsManager> assets;
    std::unique_ptr<pcs::gui::interaction::Mouse> mouse;
    std::unique_ptr<pcs::gui::working::ActionPanelRegistry> action_panels;
    std::unique_ptr<pcs::gui::working::OpenControl> open_control;
    std::unique_ptr<pcs::gui::working::AssetDetailsRegistry> asset_details;
};

auto make_app_modules() noexcept -> AppModules;

}
