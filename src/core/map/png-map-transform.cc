#include "png-map-transform.hh"

#include <cmath>
#include <numbers>

namespace pcs {

namespace {

    auto extent_x(PngMapTransformView const& view) noexcept -> double {
        return static_cast<double>(view.width) * view.resolution;
    }

    auto extent_y(PngMapTransformView const& view) noexcept -> double {
        return static_cast<double>(view.height) * view.resolution;
    }

    auto yaw_rad(PngMapTransformView const& view) noexcept -> double {
        return view.frame_config.yaw_deg * std::numbers::pi_v<double> / 180.0;
    }

}

auto make_png_map_transform_view(PngMapData const& data) noexcept -> PngMapTransformView {
    return PngMapTransformView {
        .width        = data.width,
        .height       = data.height,
        .origin_x     = data.origin_x,
        .origin_y     = data.origin_y,
        .resolution   = data.resolution,
        .plane_z      = data.plane_z,
        .frame_config = data.frame_config,
    };
}

auto png_map_origin_mode_label(PngMapOriginMode mode) noexcept -> std::string_view {
    switch (mode) {
    case PngMapOriginMode::Center:
        return "中心";
    case PngMapOriginMode::Corner1:
        return "角1";
    case PngMapOriginMode::Corner2:
        return "角2";
    case PngMapOriginMode::Corner3:
        return "角3";
    case PngMapOriginMode::Corner4:
        return "角4";
    }

    return "中心";
}

auto png_map_anchor_world(PngMapTransformView const& view) noexcept -> std::array<double, 3> {
    const auto max_x = view.origin_x + extent_x(view);
    const auto max_y = view.origin_y + extent_y(view);

    switch (view.frame_config.origin_mode) {
    case PngMapOriginMode::Center:
        return {
            view.origin_x + extent_x(view) * 0.5,
            view.origin_y + extent_y(view) * 0.5,
            view.plane_z,
        };
    case PngMapOriginMode::Corner1:
        return { view.origin_x, view.origin_y, view.plane_z };
    case PngMapOriginMode::Corner2:
        return { max_x, view.origin_y, view.plane_z };
    case PngMapOriginMode::Corner3:
        return { max_x, max_y, view.plane_z };
    case PngMapOriginMode::Corner4:
        return { view.origin_x, max_y, view.plane_z };
    }

    return { view.origin_x, view.origin_y, view.plane_z };
}

auto png_map_world_from_pixel(PngMapTransformView const& view, PixelPoint point) noexcept
    -> std::array<double, 3> {
    return {
        view.origin_x + static_cast<double>(point.x) * view.resolution,
        view.origin_y + static_cast<double>(point.y) * view.resolution,
        view.plane_z,
    };
}

auto png_map_frame_from_world(
    PngMapTransformView const& view, std::array<double, 3> const& world) noexcept
    -> std::array<double, 3> {
    const auto anchor = png_map_anchor_world(view);
    const auto dx     = world[0] - anchor[0];
    const auto dy     = world[1] - anchor[1];
    const auto yaw    = yaw_rad(view);
    const auto c      = std::cos(yaw);
    const auto s      = std::sin(yaw);

    return {
        c * dx + s * dy,
        -s * dx + c * dy,
        world[2] - view.plane_z,
    };
}

auto png_map_frame_from_pixel(PngMapTransformView const& view, PixelPoint point) noexcept
    -> std::array<double, 3> {
    return png_map_frame_from_world(view, png_map_world_from_pixel(view, point));
}

auto png_map_pixel_from_world(
    PngMapTransformView const& view, std::array<double, 3> const& world) noexcept
    -> std::optional<PixelPoint> {
    if (view.resolution <= 0.0) {
        return std::nullopt;
    }

    const auto px = static_cast<int>(std::llround((world[0] - view.origin_x) / view.resolution));
    const auto py = static_cast<int>(std::llround((world[1] - view.origin_y) / view.resolution));
    if (px < 0 || py < 0) {
        return std::nullopt;
    }

    if (static_cast<std::size_t>(px) >= view.width || static_cast<std::size_t>(py) >= view.height) {
        return std::nullopt;
    }

    return PixelPoint { px, py };
}

auto png_map_ros_origin(PngMapTransformView const& view) noexcept -> std::array<double, 3> {
    const auto lower_left_world = std::array<double, 3> { view.origin_x, view.origin_y, view.plane_z };
    const auto lower_left_frame = png_map_frame_from_world(view, lower_left_world);
    return {
        lower_left_frame[0],
        lower_left_frame[1],
        yaw_rad(view),
    };
}

}
