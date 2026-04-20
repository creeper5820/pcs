#pragma once

#include "core/assets.hh"
#include "core/runtime.hh"
#include "gui/interaction/mouse.hh"

#include <creeper-qt/layout/stacked.hh>
#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/widget/widget.hh>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace pcs::gui::working {

using namespace creeper;

struct ActionPanelContext {
    creeper::ThemeManager* manager      = nullptr;
    pcs::AssetsManager* assets          = nullptr;
    pcs::Runtime* runtime               = nullptr;
    pcs::Renderer* renderer             = nullptr;
    pcs::gui::interaction::Mouse* mouse = nullptr;

    std::function<void()> refresh_assets_list;
    std::function<void(std::string const&)> select_asset;
};

struct AssetActionPanel {
    virtual ~AssetActionPanel() = default;

    virtual auto widget() const noexcept -> QWidget*                = 0;
    virtual auto bind_asset(std::string const& id) noexcept -> void = 0;
    virtual auto clear() noexcept -> void                           = 0;
};

class ActionPanelRegistry {
public:
    using Factory =
        std::function<std::unique_ptr<AssetActionPanel>(ActionPanelContext, QFont const&)>;

    auto register_factory(pcs::AssetKind, Factory) noexcept -> void;
    auto create(pcs::AssetKind, ActionPanelContext, QFont const&) const noexcept
        -> std::unique_ptr<AssetActionPanel>;

private:
    std::unordered_map<pcs::AssetKind, Factory> factories;
};

struct ActionPanelHost {
    static constexpr auto kPlaceholderIndex = 0;

    explicit ActionPanelHost(
        ActionPanelContext context, ActionPanelRegistry const* registry, QFont const& font);

    auto widget() const noexcept -> QWidget*;
    auto clear() noexcept -> void;
    auto bind_asset(pcs::AssetKind kind, std::string const& id) noexcept -> void;

private:
    auto sync_current_panel_height() noexcept -> void;
    auto ensure_panel(pcs::AssetKind) noexcept -> AssetActionPanel*;

    ActionPanelContext context;
    ActionPanelRegistry const* registry = nullptr;
    QFont font;

    Widget* root         = nullptr;
    Stacked* stack       = nullptr;
    QWidget* placeholder = nullptr;

    std::unordered_map<pcs::AssetKind, std::unique_ptr<AssetActionPanel>> panels;
    std::unordered_map<pcs::AssetKind, int> panel_indices;
};

}
