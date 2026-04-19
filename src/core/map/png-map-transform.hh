#pragma once

#include "core/map/png-edit-ops.hh"
#include "core/map/png-map-data.hh"

#include <array>
#include <optional>
#include <string_view>

namespace pcs {

struct PngMapTransformView {
    std::size_t width  = 0;
    std::size_t height = 0;

    double origin_x   = 0.0;
    double origin_y   = 0.0;
    double resolution = 0.1;
    double plane_z    = 0.0;

    PngMapFrameConfig frame_config { };
};

auto make_png_map_transform_view(PngMapData const& data) noexcept -> PngMapTransformView;

auto png_map_origin_mode_label(PngMapOriginMode mode) noexcept -> std::string_view;

auto png_map_anchor_world(PngMapTransformView const& view) noexcept -> std::array<double, 3>;
auto png_map_world_from_pixel(PngMapTransformView const& view, PixelPoint point) noexcept
    -> std::array<double, 3>;
auto png_map_frame_from_world(
    PngMapTransformView const& view, std::array<double, 3> const& world) noexcept
    -> std::array<double, 3>;
auto png_map_frame_from_pixel(PngMapTransformView const& view, PixelPoint point) noexcept
    -> std::array<double, 3>;
auto png_map_pixel_from_world(
    PngMapTransformView const& view, std::array<double, 3> const& world) noexcept
    -> std::optional<PixelPoint>;
auto png_map_ros_origin(PngMapTransformView const& view) noexcept -> std::array<double, 3>;

}
