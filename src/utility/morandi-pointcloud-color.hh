#pragma once

namespace pcs::utility {

struct RgbColor {
    double r = 1.0;
    double g = 1.0;
    double b = 1.0;
};

auto next_morandi_pointcloud_color() noexcept -> RgbColor;

}
