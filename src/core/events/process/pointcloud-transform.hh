#pragma once

#include <expected>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace pcs::event {

struct PointcloudTransform {
    using Position = std::tuple<double, double, double>;
    using Result   = std::expected<std::vector<Position>, std::string>;

    struct Meta {
        std::string_view name = "Pointcloud Transform";
        bool recordable       = false;
        bool redoable         = true;
    };

    Meta meta { };
    std::vector<Position> points;

    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;

    double roll_deg  = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg   = 0.0;

    double pivot_x = 0.0;
    double pivot_y = 0.0;
    double pivot_z = 0.0;

    auto exec() noexcept -> Result;
    auto redo() noexcept -> Result;
};

}
