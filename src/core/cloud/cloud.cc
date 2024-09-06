#include "core/cloud/cloud.hh"
#include "core/cloud/filter.hh"
#include "core/cloud/item.hh"
#include "core/renderer/renderer.hh"

#include <spdlog/spdlog.h>

using namespace core::cloud;
namespace speed = spdlog;

struct Cloud::Impl {
public:
    using PointCloud = core::cloud::Item;

    explicit Impl() = default;

    /// CRUD
    int loadCloud(const std::string& filePath) {
        auto pointCloud = std::make_unique<PointCloud>(filePath);
        const auto index = renderer_.addCloud(*pointCloud);
        pointClouds_[index] = std::move(pointCloud);
        return index;
    }

    void saveCloud(int index, const std::string& filePath) {
        pointClouds_[index]->save(filePath);
    }

    void removeCloud(int index) {
        renderer_.removeCloud(index);
        pointClouds_.erase(index);
    }

    void removeAllCloud() {
        renderer_.removeAllCloud();
        pointClouds_.clear();
    }

    /// Modify property
    void modifyColor(int index, double r, double g, double b) {
        renderer_.modifyColor(index, r, g, b);
        pointClouds_[index]->setColor(r, g, b);
    }

    void modifyPointSize(int index, double size) {
        renderer_.modifyPointSize(index, size);
    }

    void modifyVisible(int index, bool flag) {
        renderer_.modifyVisible(index, flag);
    }

    void transformCloud(int index, Eigen::Affine3d transform) {
        renderer_.transformCloud(index, transform);
    }

    /// Select
    int addSelectCubeArea(Eigen::Vector3d corner[2]) {
    }

    int addSelectCubeArea(Eigen::Vector3d center, double length) {
    }

    int addSelectSphereArea(Eigen::Vector3d center, double radius) {
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
    }

    int extractSelectCloud(int index) {
    }

private:
    std::unordered_map<int, std::unique_ptr<PointCloud>> pointClouds_;

    core::Renderer& renderer_ = core::Renderer::instance();
    core::cloud::Filter& filter_ = core::cloud::Filter::instance();
};

Cloud::Cloud(util::Singleton<Cloud>::token)
    : pimpl_(new Impl) {
    speed::info("load core::Cloud");
}

Cloud::~Cloud() {
    delete pimpl_;
    speed::info("delete core::Cloud");
}

/// Configuration
void Cloud::connectWidget(QVTKOpenGLNativeWidget* interface) {
    auto& renderer = core::Renderer::instance();
    renderer.connectWidget(interface);
}

void Cloud::refresh() {
    auto& renderer = core::Renderer::instance();
    renderer.refresh();
}

/// CRUD
int Cloud::loadCloud(const std::string& path) {
    return pimpl_->loadCloud(path);
}
void Cloud::saveCloud(int index, const std::string& path) {
    pimpl_->saveCloud(index, path);
}
void Cloud::removeCloud(int index) {
    pimpl_->removeCloud(index);
}
void Cloud::removeAllCloud() {
    pimpl_->removeAllCloud();
}

/// Modify property
void Cloud::modifyColor(int index, double r, double g, double b) {
    pimpl_->modifyColor(index, r, g, b);
}
void Cloud::modifyPointSize(int index, double size) {
    pimpl_->modifyPointSize(index, size);
}
void Cloud::modifyVisible(int index, bool flag) {
    pimpl_->modifyVisible(index, flag);
}
void Cloud::transformCloud(int index, Eigen::Affine3d transform) {
    pimpl_->transformCloud(index, transform);
}

/// Select
int Cloud::addSelectCubeArea(Eigen::Vector3d corner[2]) {
}
int Cloud::addSelectCubeArea(Eigen::Vector3d center, double length) {
}
int Cloud::addSelectSphereArea(Eigen::Vector3d center, double radius) {
}
void Cloud::removeSelectArea(int index) {
}
void Cloud::removeAllSelectArea() {
}

/// Operators after selecting
void Cloud::removeAllSelectCloud() {
}
void Cloud::removeSelectCloud(int index) {
}
int Cloud::extractAllSelectCloud() {
}
int Cloud::extractSelectCloud(int index) {
}