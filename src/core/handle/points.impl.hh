#pragma once

#include "core/handle/points.hh"
#include "core/units/points.hh"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcs;

using Point      = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

constexpr auto error_code = -1;

struct PointsHandle::Impl final {

    std::unique_ptr<PointsUnit> unit;
    std::shared_ptr<PointCloud> points;

    auto load_from_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view> {
        points = std::make_shared<PointCloud>();
        if (error_code == pcl::io::loadPCDFile(path, *points)) {
            return std::unexpected { "Failed to read pointcloud from filesystem" };
        }
        unit = std::make_unique<PointsUnit>(points->points);
        return {};
    }

    auto save_into_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view> {
        if (points == nullptr) {
            return std::unexpected { "Points is not loaded" };
        }
        if (error_code == pcl::io::savePCDFile(path, *points, false)) {
            return std::unexpected { "Failed to save pointcloud to filesystem" };
        }
        return {};
    }
};
