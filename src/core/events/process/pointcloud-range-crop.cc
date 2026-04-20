#include "pointcloud-range-crop.hh"

namespace pcs::event {

auto PointcloudRangeCrop::exec() noexcept -> Result {
    if (points.empty()) {
        return std::unexpected { "点云数据为空" };
    }
    if (x_min > x_max || y_min > y_max || z_min > z_max) {
        return std::unexpected { "范围参数无效" };
    }

    auto cropped = std::vector<Position> { };
    cropped.reserve(points.size());

    for (auto const& point : points) {
        const auto x = std::get<0>(point);
        const auto y = std::get<1>(point);
        const auto z = std::get<2>(point);

        if (x < x_min || x > x_max || y < y_min || y > y_max || z < z_min || z > z_max) {
            continue;
        }

        cropped.push_back(point);
    }

    if (cropped.empty()) {
        return std::unexpected { "截取后点云为空" };
    }

    return cropped;
}

auto PointcloudRangeCrop::redo() noexcept -> Result { return exec(); }

}
