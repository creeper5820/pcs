#pragma once

#include "core/handle/points.hh"
#include "core/units/points.hh"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cstdint>

using namespace pcs;

using Point      = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

constexpr auto error_code = -1;

struct PointsHandle::Impl final {

    std::unique_ptr<PointsUnit> unit;
    std::shared_ptr<PointCloud> points;
    bool coordinate_visible = true;
    bool visible            = true;
    std::vector<Renderer*> attached_renderers;

    auto rebind_unit(std::unique_ptr<PointsUnit> next_unit) noexcept -> void {
        if (unit != nullptr) {
            for (auto* renderer : attached_renderers) {
                if (renderer != nullptr) {
                    renderer->detach(*unit);
                }
            }
        }

        unit = std::move(next_unit);
        if (unit == nullptr) {
            return;
        }

        unit->set_visibility(visible);
        unit->set_coordinate_visibility(visible && coordinate_visible);

        for (auto* renderer : attached_renderers) {
            if (renderer != nullptr) {
                renderer->attach(*unit);
            }
        }
    }

    auto load_from_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view> {
        points = std::make_shared<PointCloud>();
        if (error_code == pcl::io::loadPCDFile(path, *points)) {
            return std::unexpected { "Failed to read pointcloud from filesystem" };
        }

        auto next_unit = std::make_unique<PointsUnit>(points->points);
        rebind_unit(std::move(next_unit));
        return { };
    }

    auto load_from_positions(std::vector<PointsHandle::Position> const& positions) noexcept
        -> std::expected<void, std::string_view> {
        points = std::make_shared<PointCloud>();
        points->points.reserve(positions.size());

        for (const auto& [x, y, z] : positions) {
            points->points.emplace_back(
                static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
        }

        points->width  = static_cast<std::uint32_t>(points->points.size());
        points->height = 1;

        auto next_unit = std::make_unique<PointsUnit>(points->points);
        rebind_unit(std::move(next_unit));
        return { };
    }

    auto save_into_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view> {
        if (points == nullptr) {
            return std::unexpected { "Points is not loaded" };
        }
        if (error_code == pcl::io::savePCDFile(path, *points, false)) {
            return std::unexpected { "Failed to save pointcloud to filesystem" };
        }
        return { };
    }

    auto get_positions() const noexcept -> std::vector<PointsHandle::Position> {
        auto result = std::vector<PointsHandle::Position> { };
        if (points == nullptr) {
            return result;
        }

        result.reserve(points->points.size());
        for (const auto& point : points->points) {
            result.emplace_back(point.x, point.y, point.z);
        }

        return result;
    }

    auto set_coordinate_visibility(bool on) noexcept -> void {
        coordinate_visible = on;
        if (unit != nullptr) {
            unit->set_coordinate_visibility(visible && on);
        }
    }

    auto set_visibility(bool on) noexcept -> void {
        visible = on;
        if (unit != nullptr) {
            unit->set_visibility(on);
            unit->set_coordinate_visibility(on && coordinate_visible);
        }
    }

    auto attach_renderer(Renderer& renderer) noexcept -> void {
        if (std::find(attached_renderers.begin(), attached_renderers.end(), &renderer)
            == attached_renderers.end()) {
            attached_renderers.push_back(&renderer);
        }

        if (unit != nullptr) {
            renderer.attach(*unit);
        }
    }

    auto detach_renderer(Renderer& renderer) noexcept -> void {
        if (unit != nullptr) {
            renderer.detach(*unit);
        }

        std::erase(attached_renderers, &renderer);
    }

    auto coordinate_visibility() const noexcept -> bool { return coordinate_visible; }
};
