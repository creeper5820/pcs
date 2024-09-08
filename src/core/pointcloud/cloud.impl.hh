#pragma once

#include "core/pointcloud/cloud.hh"
#include "core/renderer/renderer.hh"
#include "core/share/cloud-box.hh"

#include <spdlog/spdlog.h>

namespace core::cloud {
struct CloudManager::Impl {
public:
    /// CRUD
    std::unique_ptr<CloudPackage> makePackage(const std::string& path) {
        auto source = std::make_unique<CloudSource>(path);
        auto object = renderer_.addCloud(*source);
        return std::make_unique<CloudPackage>(std::move(source), std::move(object));
    }

    /// Select
    int addSelectCubeArea(Eigen::Vector3d corner[2]) {
        return {};
    }

    int addSelectCubeArea(Eigen::Vector3d center, double length) {
        return {};
    }

    int addSelectSphereArea(Eigen::Vector3d center, double radius) {
        return {};
    }

    void removeAllSelectArea() {
    }

    void removeSelectArea(int index) {
    }

    /// Operators after selecting
    void removeAllSelectCloud() {
    }

    void removeSelectCloud(int index) {
    }

    int extractAllSelectCloud() {
        return {};
    }

    int extractSelectCloud(int index) {
        return {};
    }

private:
    std::unordered_map<CloudObject, std::unique_ptr<CloudSource>> pointClouds_;

    core::Renderer& renderer_ = core::Renderer::instance();
};
}