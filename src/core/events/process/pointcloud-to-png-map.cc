#include "pointcloud-to-png-map.hh"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_set>

#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {

using Point      = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

struct PairHash {
    auto operator()(std::pair<std::size_t, std::size_t> const& p) const noexcept -> std::size_t {
        return std::hash<std::size_t> { }(p.first) ^ (std::hash<std::size_t> { }(p.second) << 1U);
    }
};

struct ObstacleNode {
    std::uint8_t value { 255 };
    std::unordered_set<std::int16_t> height_table;

    auto update_height(double height) noexcept -> void {
        const auto key = static_cast<std::int16_t>(std::round(height * 100.0));
        height_table.insert(key);
    }

    auto maximum_height_range() const noexcept -> double {
        if (height_table.empty()) {
            return 0.0;
        }

        const auto [min_value, max_value] =
            std::minmax_element(height_table.begin(), height_table.end());
        return static_cast<double>(*max_value - *min_value) / 100.0;
    }
};

struct ObstacleMap {
    std::vector<std::vector<ObstacleNode>> nodes;

    explicit ObstacleMap(std::size_t width, std::size_t height)
        : nodes(width, std::vector<ObstacleNode>(height)) { }

    auto width() const noexcept -> std::size_t { return nodes.size(); }
    auto height() const noexcept -> std::size_t { return nodes.empty() ? 0 : nodes.front().size(); }

    auto at(std::size_t x, std::size_t y) noexcept -> ObstacleNode& { return nodes[x][y]; }
    auto at(std::size_t x, std::size_t y) const noexcept -> ObstacleNode const& {
        return nodes[x][y];
    }

    auto update_with_round_area(std::size_t x, std::size_t y, std::size_t expand,
        std::function<void(std::size_t, std::size_t, ObstacleNode&)> const& apply) noexcept
        -> void {
        apply(x, y, at(x, y));

        const auto x_min = x >= expand ? x - expand : 0;
        const auto y_min = y >= expand ? y - expand : 0;
        const auto x_max = std::min(x + expand, width() - 1);
        const auto y_max = std::min(y + expand, height() - 1);

        const auto expand_sq =
            static_cast<std::int64_t>(expand) * static_cast<std::int64_t>(expand);

        for (auto xi = x_min; xi <= x_max; ++xi) {
            for (auto yi = y_min; yi <= y_max; ++yi) {
                const auto dx = static_cast<std::int64_t>(xi) - static_cast<std::int64_t>(x);
                const auto dy = static_cast<std::int64_t>(yi) - static_cast<std::int64_t>(y);
                if (dx * dx + dy * dy > expand_sq) {
                    continue;
                }
                apply(xi, yi, at(xi, yi));
            }
        }
    }
};

auto as_pointcloud(std::vector<pcs::event::ConvertPointcloudToPngMap::Position> const& points)
    -> std::shared_ptr<PointCloud> {
    auto cloud = std::make_shared<PointCloud>();
    cloud->points.reserve(points.size());

    for (const auto& [x, y, z] : points) {
        cloud->points.emplace_back(
            static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    }

    cloud->width  = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;

    return cloud;
}

auto remove_outlier_points(std::shared_ptr<PointCloud> const& pointcloud, double resolution)
    -> void {
    if (pointcloud == nullptr || pointcloud->empty()) {
        return;
    }

    auto outlier_filter = pcl::StatisticalOutlierRemoval<Point> { };
    outlier_filter.setMeanK(20);
    outlier_filter.setStddevMulThresh(0.5);
    outlier_filter.setInputCloud(pointcloud);
    outlier_filter.filter(*pointcloud);

    auto voxel = pcl::VoxelGrid<Point> { };
    voxel.setLeafSize(static_cast<float>(resolution), static_cast<float>(resolution),
        static_cast<float>(resolution));
    voxel.setInputCloud(pointcloud);
    voxel.filter(*pointcloud);
}

auto generate_png_map(PointCloud const& pointcloud, pcs::PngMapParameters const& params,
    double minimum_z) -> std::expected<pcs::PngMapData, std::string> {
    if (pointcloud.empty()) {
        return std::unexpected { "过滤后点云为空" };
    }

    auto point_min = Point { };
    auto point_max = Point { };
    pcl::getMinMax3D(pointcloud, point_min, point_max);

    const auto to_index = [resolution = params.resolution](double value) -> std::size_t {
        return static_cast<std::size_t>(std::max(value, 0.0) / resolution);
    };

    const auto width  = to_index(point_max.x - point_min.x) + 1;
    const auto height = to_index(point_max.y - point_min.y) + 1;

    if (width == 0 || height == 0) {
        return std::unexpected { "生成的地图尺寸无效" };
    }

    auto map = ObstacleMap { width, height };

    const auto sample_expand = static_cast<std::size_t>(0.1 / params.resolution);
    auto visited = std::unordered_set<std::pair<std::size_t, std::size_t>, PairHash> { };

    const auto area_start = minimum_z + params.z_area_start;
    const auto area_end   = minimum_z + params.z_area_end;
    const auto plane_z    = area_start;
    const auto area_min   = std::min(area_start, area_end);
    const auto area_max   = std::max(area_start, area_end);

    for (auto const& point : pointcloud) {
        if (point.z < area_min || point.z > area_max) {
            continue;
        }

        auto x = to_index(point.x - point_min.x);
        auto y = to_index(point.y - point_min.y);

        x = std::min(x, width - 1);
        y = std::min(y, height - 1);

        map.update_with_round_area(
            x, y, sample_expand, [&](std::size_t xi, std::size_t yi, ObstacleNode& node) {
                node.update_height(static_cast<double>(point.z));
                visited.insert(std::make_pair(xi, yi));
            });
    }

    if (visited.empty()) {
        return std::unexpected { "所选 Z 区间内无有效点" };
    }

    const auto influence_expand =
        static_cast<std::size_t>(std::round(params.influence_radius / params.resolution));

    for (const auto& [x, y] : visited) {
        auto& node = map.at(x, y);
        if (node.height_table.size() < params.points_limit) {
            continue;
        }
        if (node.maximum_height_range() < params.height_limit) {
            continue;
        }

        map.update_with_round_area(x, y, influence_expand,
            [](std::size_t, std::size_t, ObstacleNode& target) { target.value = 0; });
    }

    auto pixels = std::vector<std::uint8_t>(width * height, 255);
    for (std::size_t x = 0; x < width; ++x) {
        for (std::size_t y = 0; y < height; ++y) {
            pixels[y * width + x] = map.at(x, y).value;
        }
    }

    auto data         = pcs::PngMapData { };
    data.pixels       = std::move(pixels);
    data.width        = width;
    data.height       = height;
    data.origin_x     = point_min.x;
    data.origin_y     = point_min.y;
    data.resolution   = params.resolution;
    data.plane_z      = plane_z;
    data.z_area_start = area_start;
    data.z_area_end   = area_end;

    return data;
}

}

namespace pcs::event {

auto ConvertPointcloudToPngMap::runtime_exec(std::unique_ptr<Context> context) noexcept -> Result {
    if (context == nullptr || context->points.empty()) {
        return std::unexpected { "点云数据未加载" };
    }

    auto parameters = context->parameters;

    if (parameters.resolution < 0.01) {
        return std::unexpected { "分辨率必须大于等于 0.01" };
    }
    if (parameters.points_limit == 0) {
        return std::unexpected { "有效点云数必须大于 0" };
    }
    if (parameters.height_limit < 0.0) {
        return std::unexpected { "有效高度差必须大于等于 0" };
    }
    if (parameters.influence_radius < 0.0) {
        return std::unexpected { "影响半径必须大于等于 0" };
    }
    if (parameters.z_area_start < 0.0) {
        return std::unexpected { "Z 区间起点必须大于等于 0" };
    }
    if (parameters.z_area_end < 0.0) {
        return std::unexpected { "Z 区间终点必须大于等于 0" };
    }

    auto pointcloud = as_pointcloud(context->points);

    auto original_min = Point { };
    auto original_max = Point { };
    pcl::getMinMax3D(*pointcloud, original_min, original_max);

    auto filtered = std::make_shared<PointCloud>(*pointcloud);
    remove_outlier_points(filtered, parameters.resolution);

    if (filtered->empty()) {
        return std::unexpected { "离群点过滤后点云为空" };
    }

    return generate_png_map(*filtered, parameters, original_min.z);
}

}
