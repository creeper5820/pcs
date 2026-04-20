#pragma once

#include <expected>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace pcs::event {

struct PointcloudRangeCrop {
    using Position = std::tuple<double, double, double>;
    using Result   = std::expected<std::vector<Position>, std::string>;

    struct Meta {
        std::string_view name = "Pointcloud Range Crop";
        bool recordable       = false;
        bool redoable         = true;
    };

    Meta meta { };
    std::vector<Position> points;
    double x_min = -10.0;
    double x_max = 10.0;
    double y_min = -10.0;
    double y_max = 10.0;
    double z_min = -10.0;
    double z_max = 10.0;

    auto exec() noexcept -> Result;
    auto redo() noexcept -> Result;
};

}
