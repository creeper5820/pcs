#pragma once

#include "core/assets.hh"

#include <creeper-qt/layout/stacked.hh>
#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/utility/wrapper/mutable-value.hh>
#include <creeper-qt/widget/buttons/outlined-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/text-fields.hh>
#include <creeper-qt/widget/widget.hh>

#include <array>
#include <functional>
#include <memory>
#include <string>

namespace pcs::gui::working {

using namespace creeper;

struct ActionPanelContext {
    creeper::ThemeManager* manager = nullptr;
    pcs::AssetsManager* assets     = nullptr;

    std::function<void()> refresh_assets_list;
    std::function<void(std::string const&)> select_asset;
};

struct AssetActionPanel {
    virtual ~AssetActionPanel() = default;

    virtual auto widget() const noexcept -> QWidget*                = 0;
    virtual auto bind_asset(std::string const& id) noexcept -> void = 0;
    virtual auto clear() noexcept -> void                           = 0;
};

struct PointcloudActionPanel final : AssetActionPanel {
    explicit PointcloudActionPanel(ActionPanelContext context, QFont const& font);

    auto widget() const noexcept -> QWidget* override;
    auto bind_asset(std::string const& id) noexcept -> void override;
    auto clear() noexcept -> void override;

private:
    ActionPanelContext context;
    std::string selected_asset_id;

    std::array<std::shared_ptr<MutableDouble>, 4> color_channels {
        std::make_shared<MutableDouble>(),
        std::make_shared<MutableDouble>(),
        std::make_shared<MutableDouble>(),
        std::make_shared<MutableDouble>(),
    };

    text_field::internal::BasicTextField* resolution_input       = nullptr;
    text_field::internal::BasicTextField* points_limit_input     = nullptr;
    text_field::internal::BasicTextField* height_limit_input     = nullptr;
    text_field::internal::BasicTextField* influence_radius_input = nullptr;
    text_field::internal::BasicTextField* z_area_start_input     = nullptr;
    text_field::internal::BasicTextField* z_area_end_input       = nullptr;

    OutlinedButton* save_button     = nullptr;
    OutlinedButton* generate_button = nullptr;
    FilledCard* root                = nullptr;
};

struct ModelActionPanel final : AssetActionPanel {
    explicit ModelActionPanel(ActionPanelContext context, QFont const& font);

    auto widget() const noexcept -> QWidget* override;
    auto bind_asset(std::string const& id) noexcept -> void override;
    auto clear() noexcept -> void override;

private:
    ActionPanelContext context;
    std::string selected_asset_id;

    text_field::internal::BasicTextField* density_input         = nullptr;
    text_field::internal::BasicTextField* sample_distance_input = nullptr;
    text_field::internal::BasicTextField* unit_scale_input      = nullptr;
    text_field::internal::BasicTextField* max_points_input      = nullptr;

    OutlinedButton* convert_button = nullptr;
    FilledCard* root               = nullptr;
};

struct PngMapActionPanel final : AssetActionPanel {
    explicit PngMapActionPanel(ActionPanelContext context, QFont const& font);

    auto widget() const noexcept -> QWidget* override;
    auto bind_asset(std::string const& id) noexcept -> void override;
    auto clear() noexcept -> void override;

private:
    ActionPanelContext context;
    std::string selected_asset_id;

    OutlinedButton* save_button = nullptr;
    FilledCard* root            = nullptr;
};

struct ActionPanelHost {
    static constexpr auto kPlaceholderIndex = 0;
    static constexpr auto kPointcloudIndex  = 1;
    static constexpr auto kModelIndex       = 2;
    static constexpr auto kPngMapIndex      = 3;

    explicit ActionPanelHost(ActionPanelContext context, QFont const& font);

    auto widget() const noexcept -> QWidget*;
    auto clear() noexcept -> void;
    auto bind_asset(pcs::AssetKind kind, std::string const& id) noexcept -> void;

private:
    auto sync_current_panel_height() noexcept -> void;

    Widget* root   = nullptr;
    Stacked* stack = nullptr;

    std::unique_ptr<AssetActionPanel> pointcloud_panel;
    std::unique_ptr<AssetActionPanel> model_panel;
    std::unique_ptr<AssetActionPanel> png_map_panel;

    QWidget* placeholder = nullptr;
};

}
