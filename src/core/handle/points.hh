#pragma once

#include "core/renderer.hh"
#include "utility/pimpl.hh"

#include <expected>
#include <vector>

namespace pcs {

struct PointsHandle {
    PCS_PIMPL_DEFINITION(PointsHandle)

public:
    using Position = std::tuple<double, double, double>;

    auto set_position(double x, double y, double z) noexcept -> void;
    auto get_position() const noexcept -> Position;

    auto set_overall_color(double r, double g, double b) noexcept -> void;
    auto get_overall_color() const noexcept -> std::tuple<double, double, double>;

    auto get_points_size() const noexcept -> std::size_t;

    auto set_visibility(bool) noexcept -> void;

    auto load_from_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view>;

    auto load_from_positions(std::vector<Position> const& points) noexcept
        -> std::expected<void, std::string_view>;

    auto save_into_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view>;

    auto attach_renderer(Renderer&) noexcept -> void;

    auto detach_renderer(Renderer&) noexcept -> void;
};

}
