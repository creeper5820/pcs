#include "pointcloud-transform.hh"

#include <array>
#include <cmath>

namespace pcs::event {

namespace {

constexpr auto kPi = 3.14159265358979323846;

auto deg_to_rad(double value) noexcept -> double { return value * kPi / 180.0; }

}

auto PointcloudTransform::exec() noexcept -> Result {
    if (points.empty()) {
        return std::unexpected { "点云数据为空" };
    }

    const auto roll  = deg_to_rad(roll_deg);
    const auto pitch = deg_to_rad(pitch_deg);
    const auto yaw   = deg_to_rad(yaw_deg);

    const auto cr = std::cos(roll);
    const auto sr = std::sin(roll);
    const auto cp = std::cos(pitch);
    const auto sp = std::sin(pitch);
    const auto cy = std::cos(yaw);
    const auto sy = std::sin(yaw);

    const auto r00 = cy * cp;
    const auto r01 = cy * sp * sr - sy * cr;
    const auto r02 = cy * sp * cr + sy * sr;
    const auto r10 = sy * cp;
    const auto r11 = sy * sp * sr + cy * cr;
    const auto r12 = sy * sp * cr - cy * sr;
    const auto r20 = -sp;
    const auto r21 = cp * sr;
    const auto r22 = cp * cr;

    auto transformed = std::vector<Position> { };
    transformed.reserve(points.size());

    for (auto const& point : points) {
        const auto x = std::get<0>(point) - pivot_x;
        const auto y = std::get<1>(point) - pivot_y;
        const auto z = std::get<2>(point) - pivot_z;

        const auto rx = r00 * x + r01 * y + r02 * z;
        const auto ry = r10 * x + r11 * y + r12 * z;
        const auto rz = r20 * x + r21 * y + r22 * z;

        transformed.emplace_back(
            rx + pivot_x + tx, ry + pivot_y + ty, rz + pivot_z + tz);
    }

    return transformed;
}

auto PointcloudTransform::redo() noexcept -> Result { return exec(); }

}
