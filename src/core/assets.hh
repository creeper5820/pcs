#pragma once

#include "core/handle/model.hh"
#include "core/handle/png-map.hh"
#include "core/handle/points.hh"
#include "core/map/model-pointcloud-data.hh"
#include "core/map/png-map-data.hh"
#include "core/renderer.hh"

#include "utility/pimpl.hh"

#include <expected>
#include <generator>
#include <optional>
#include <tuple>
#include <vector>

namespace pcs {

enum class AssetKind {
    Pointcloud,
    Model,
    PngMap,
};

class AssetsManager final {
    PCS_PIMPL_DEFINITION(AssetsManager);

public:
    explicit AssetsManager(Renderer&) noexcept;

    auto update_renderer() const noexcept -> void;

    auto clean_assets() noexcept -> void;

    auto get_asset_ids() const noexcept -> std::generator<std::string_view>;
    auto get_asset_kind(std::string const& id) const noexcept -> std::optional<AssetKind>;
    auto get_asset_display_name(std::string const& id) const noexcept -> std::optional<std::string>;
    auto get_asset_name(std::string const& id) const noexcept -> std::optional<std::string>;
    auto get_asset_path(std::string const& id) const noexcept -> std::optional<std::string>;
    auto is_asset_visible(std::string const& id) const noexcept -> std::optional<bool>;

    auto get_pointcloud_handle(std::string const& id) noexcept -> std::optional<PointsHandle*>;
    auto get_model_handle(std::string const& id) noexcept -> std::optional<ModelHandle*>;
    auto get_png_map_handle(std::string const& id) noexcept -> std::optional<PngMapHandle*>;

    auto register_pointcloud_asset(std::unique_ptr<PointsHandle>, std::string const& name,
        std::string const& location, bool persisted = true) noexcept -> std::string;
    auto register_model_asset(
        std::unique_ptr<ModelHandle>, std::string const& name, std::string const& location) noexcept
        -> std::string;
    auto register_png_map_asset(std::unique_ptr<PngMapHandle>, std::string const& name,
        std::string const& location, bool persisted = true) noexcept -> std::string;

    auto set_asset_visibility(std::string const& id, bool on) noexcept -> bool;

    auto convert_model_to_pointcloud(
        std::string const& id, ModelToPointcloudParameters const& = { }) noexcept
        -> std::expected<std::string, std::string>;

    auto upsert_generated_pointcloud(std::string const& source_id,
        std::vector<std::tuple<double, double, double>> const& points) noexcept
        -> std::expected<std::string, std::string>;

    auto generate_png_map_from_pointcloud(std::string const& id, PngMapParameters const&) noexcept
        -> std::expected<std::string, std::string>;

    auto upsert_generated_png_map(std::string const& source_id, PngMapData const&) noexcept
        -> std::expected<std::string, std::string>;

    auto create_png_map_asset_from_data(std::string const& source_id, PngMapData const&) noexcept
        -> std::expected<std::string, std::string>;

    auto replace_png_map_asset_data(std::string const& id, PngMapData const&) noexcept
        -> std::expected<void, std::string>;

    auto save_pointcloud_asset(std::string const& id, std::string const& path) noexcept
        -> std::expected<void, std::string>;

    auto save_png_map_asset(std::string const& id, std::string const& path) noexcept
        -> std::expected<void, std::string>;

    auto export_png_map_asset(std::string const& id, std::string const& directory) noexcept
        -> std::expected<void, std::string>;

    auto remove_asset(std::string const& id) noexcept -> bool;

    auto set_default_point_color(double, double, double) noexcept -> void;
};

}
