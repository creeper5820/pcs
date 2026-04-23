#pragma once

#include "core/renderer.hh"
#include "utility/pimpl.hh"

#include <expected>
#include <optional>
#include <string_view>
#include <tuple>
#include <vector>

namespace pcs {

struct PointsHandle {
    PCS_PIMPL_DEFINITION(PointsHandle)

public:
    static constexpr std::string_view kKind = "pointcloud";

    using Position = std::tuple<double, double, double>;

    auto set_position(double x, double y, double z) noexcept -> void;
    auto get_position() const noexcept -> Position;

    auto set_overall_color(double r, double g, double b, double a = 1.0) noexcept -> void;
    auto get_overall_color() const noexcept -> std::tuple<double, double, double, double>;

    auto get_points_size() const noexcept -> std::size_t;

    auto get_positions() const noexcept -> std::vector<Position>;

    auto pick_position(Renderer&, int display_x, int display_y) const noexcept
        -> std::optional<Position>;

    auto set_visibility(bool) noexcept -> void;
    auto set_coordinate_visibility(bool) noexcept -> void;
    auto coordinate_visibility() const noexcept -> bool;

    auto load_from_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view>;

    auto load_from_positions(std::vector<Position> const& points) noexcept
        -> std::expected<void, std::string_view>;

    auto save_into_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view>;

    auto clone(std::string const& target_name) const noexcept
        -> std::expected<std::unique_ptr<PointsHandle>, std::string>;

    auto attach_renderer(Renderer&) noexcept -> void;

    auto detach_renderer(Renderer&) noexcept -> void;
};

}
