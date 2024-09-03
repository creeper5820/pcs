#include "core/cloud/cloud.hh"
#include "core/cloud/item.hh"
#include "core/renderer/renderer.hh"

#include <spdlog/spdlog.h>

using namespace core::cloud;
namespace speed = spdlog;

struct Cloud::Impl {
public:
    explicit Impl() = default;

private:
    std::unordered_map<std::string, Item> items_;
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

/// CRUD
bool Cloud::loadCloud(int index, const std::string& path) {
}
void Cloud::saveCloud(int index, const std::string& path) {
}
void Cloud::removeCloud(int index) {
}
void Cloud::removeAllCloud() {
}

/// Modify property
void Cloud::modifyColor(int index, double r, double g, double b) {
}
void Cloud::modifyPointSize(int index, double size) {
}
void Cloud::modifyVisible(int index, bool flag) {
}

void Cloud::transformCloud(int index, Eigen::Affine3d transform) {
}

/// Select
void Cloud::addSelectCubeArea(int index, Eigen::Vector3d corner[2]) {
}
void Cloud::addSelectCubeArea(int index, Eigen::Vector3d center, double length) {
}
void Cloud::addSelectSphereArea(int index, Eigen::Vector3d center, double radius) {
}

void Cloud::removeSelectArea(int index) {
}
void Cloud::removeAllSelectArea() {
}

/// Operators after selecting
void Cloud::removeSelectCloud() {
}
void Cloud::removeSelectCloud(int index) {
}

void Cloud::segmentSelectCloud() {
}
void Cloud::segmentSelectCloud(int index) {
}

void Cloud::extractSelectCloud() {
}
void Cloud::extractSelectCloud(int index) {
}