#pragma once
#include "core/handle/points.hh"
#include "core/renderer.hh"

#include "utility/pimpl.hh"

#include <generator>

namespace pcs {

class AssetsManager final {
    PCS_PIMPL_DEFINITION(AssetsManager);

public:
    explicit AssetsManager(Renderer&) noexcept;

    auto update_renderer() const noexcept -> void;

    auto open_pointcloud_file(std::string const&) noexcept -> void;

    auto clean_pointclouds() noexcept -> void;

    ///
    /// Getter
    ///
    auto get_pointcloud_handle(std::string const& location) noexcept
        -> std::optional<PointsHandle*>;

    auto get_pointcloud_locations() const noexcept -> std::generator<std::string_view>;

    ///
    /// Setter
    ///
    auto set_use_default_appearence(std::string const&, bool) noexcept -> void;

    auto set_default_point_color(double, double, double) noexcept -> void;

    auto set_pointclouds_visibility(bool) noexcept -> void;
};

}
