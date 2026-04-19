#include "core/map/png-edit-ops.hh"

#include <algorithm>
#include <cmath>

namespace pcs {

namespace {

    auto put_pixel(std::vector<std::uint8_t>& pixels, std::size_t width, std::size_t height, int x,
        int y, std::uint8_t value) noexcept -> void {
        if (x < 0 || y < 0) {
            return;
        }

        if (static_cast<std::size_t>(x) >= width || static_cast<std::size_t>(y) >= height) {
            return;
        }

        pixels[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)] = value;
    }

    auto draw_disk(std::vector<std::uint8_t>& pixels, std::size_t width, std::size_t height,
        PixelPoint center, int radius, std::uint8_t value) noexcept -> void {
        if (radius <= 0) {
            put_pixel(pixels, width, height, center.x, center.y, value);
            return;
        }

        const auto radius_sq = radius * radius;
        for (auto dy = -radius; dy <= radius; ++dy) {
            for (auto dx = -radius; dx <= radius; ++dx) {
                if (dx * dx + dy * dy > radius_sq) {
                    continue;
                }

                put_pixel(pixels, width, height, center.x + dx, center.y + dy, value);
            }
        }
    }

}

auto draw_line_with_thickness(std::vector<std::uint8_t>& pixels, std::size_t width,
    std::size_t height, PixelPoint from, PixelPoint to, std::size_t thickness,
    std::uint8_t value) noexcept -> void {
    if (pixels.empty() || width == 0 || height == 0) {
        return;
    }

    const auto line_thickness = std::max<std::size_t>(thickness, 1);
    const auto radius         = static_cast<int>((line_thickness - 1U) / 2U);

    auto x0 = from.x;
    auto y0 = from.y;
    auto x1 = to.x;
    auto y1 = to.y;

    auto dx  = std::abs(x1 - x0);
    auto sx  = x0 < x1 ? 1 : -1;
    auto dy  = -std::abs(y1 - y0);
    auto sy  = y0 < y1 ? 1 : -1;
    auto err = dx + dy;

    while (true) {
        draw_disk(pixels, width, height, PixelPoint { x0, y0 }, radius, value);

        if (x0 == x1 && y0 == y1) {
            break;
        }

        const auto e2 = err * 2;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

auto nearest_point_index(std::vector<PixelPoint> const& points, PixelPoint target,
    double max_distance) noexcept -> std::optional<std::size_t> {
    if (points.empty() || max_distance < 0.0) {
        return std::nullopt;
    }

    auto nearest      = std::optional<std::size_t> { };
    auto best_sq_dist = max_distance * max_distance;

    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto dx = static_cast<double>(points[i].x - target.x);
        const auto dy = static_cast<double>(points[i].y - target.y);
        const auto sq = dx * dx + dy * dy;

        if (sq > best_sq_dist) {
            continue;
        }

        best_sq_dist = sq;
        nearest      = i;
    }

    return nearest;
}

}
