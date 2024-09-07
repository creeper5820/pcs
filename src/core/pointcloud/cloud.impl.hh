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

    void saveCloud(StereoIndex index, const std::string& filePath) {
        // pointClouds_[index]->save(filePath);
    }

    void removeCloud(StereoIndex index) {
        renderer_.removeStereoProps(index);
        // pointClouds_.erase(index);
    }

    void removePackage(CloudPackage& package) {
        renderer_.removeObject(*package.object);
    }

    void removeAllCloud() {
        renderer_.removeAllProps();
        pointClouds_.clear();
    }

    /// Modify property
    void modifyColor(StereoIndex index, double r, double g, double b) {
        renderer_.modifyColor(index, r, g, b);
        // pointClouds_[index]->setColor(r, g, b);
    }

    void modifyPointSize(StereoIndex index, double size) {
        renderer_.modifyPointSize(index, size);
    }

    void modifyVisible(StereoIndex index, bool flag) {
        renderer_.modifyVisible(index, flag);
    }

    void transformCloud(StereoIndex index, Eigen::Affine3d transform) {
        renderer_.transformCloud(index, transform);
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