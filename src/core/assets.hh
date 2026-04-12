#pragma once

#include "core/handle/model.hh"
#include "core/handle/points.hh"
#include "core/renderer.hh"

#include "utility/pimpl.hh"

#include <expected>
#include <generator>
#include <optional>

namespace pcs {

enum class AssetKind {
    Pointcloud,
    Model,
};

class AssetsManager final {
    PCS_PIMPL_DEFINITION(AssetsManager);

public:
    explicit AssetsManager(Renderer&) noexcept;

    auto update_renderer() const noexcept -> void;

    auto open_file(std::string const& location) noexcept -> void;

    auto clean_assets() noexcept -> void;

    auto get_asset_ids() const noexcept -> std::generator<std::string_view>;
    auto get_asset_kind(std::string const& id) const noexcept -> std::optional<AssetKind>;
    auto get_asset_display_name(std::string const& id) const noexcept -> std::optional<std::string>;
    auto get_asset_name(std::string const& id) const noexcept -> std::optional<std::string>;
    auto get_asset_path(std::string const& id) const noexcept -> std::optional<std::string>;
    auto is_asset_visible(std::string const& id) const noexcept -> std::optional<bool>;

    auto get_pointcloud_handle(std::string const& id) noexcept -> std::optional<PointsHandle*>;
    auto get_model_handle(std::string const& id) noexcept -> std::optional<ModelHandle*>;

    auto set_asset_visibility(std::string const& id, bool on) noexcept -> bool;

    auto convert_model_to_pointcloud(std::string const& id) noexcept
        -> std::expected<std::string, std::string>;

    auto save_pointcloud_asset(std::string const& id, std::string const& path) noexcept
        -> std::expected<void, std::string>;

    auto set_default_point_color(double, double, double) noexcept -> void;
};

}
