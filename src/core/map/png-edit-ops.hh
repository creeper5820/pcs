#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace pcs {

struct PixelPoint {
    int x = 0;
    int y = 0;
};

auto draw_line_with_thickness(std::vector<std::uint8_t>& pixels, std::size_t width,
    std::size_t height, PixelPoint from, PixelPoint to, std::size_t thickness,
    std::uint8_t value) noexcept -> void;

auto nearest_point_index(std::vector<PixelPoint> const& points, PixelPoint target,
    double max_distance) noexcept -> std::optional<std::size_t>;

}
