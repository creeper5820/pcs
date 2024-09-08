#include "core/pointcloud/cloud.hh"
#include "core/pointcloud/cloud.impl.hh"

using namespace core::cloud;

CloudManager::CloudManager(util::Singleton<CloudManager>::token)
    : pimpl_(new Impl) {
    spdlog::info("load CloudManager");
}

CloudManager::~CloudManager() {
    delete pimpl_;
    spdlog::info("delete CloudManager");
}

/// Configuration
void CloudManager::connectWidget(QVTKOpenGLNativeWidget* interface) {
    auto& renderer = core::Renderer::instance();
    renderer.connectWidget(interface);
}

void CloudManager::refresh() {
    auto& renderer = core::Renderer::instance();
    renderer.refresh();
}

std::unique_ptr<CloudPackage> CloudManager::makePackage(const std::string& path) {
    return pimpl_->makePackage(path);
}

/// Select
int CloudManager::addSelectCubeArea(Eigen::Vector3d corner[2]) {
    return {};
}
int CloudManager::addSelectCubeArea(Eigen::Vector3d center, double length) {
    return {};
}
int CloudManager::addSelectSphereArea(Eigen::Vector3d center, double radius) {
    return {};
}
void CloudManager::removeSelectArea(int index) {
}
void CloudManager::removeAllSelectArea() {
}

/// Operators after selecting
void CloudManager::removeAllSelectCloud() {
}
void CloudManager::removeSelectCloud(int index) {
}
int CloudManager::extractAllSelectCloud() {
    return {};
}
int CloudManager::extractSelectCloud(int index) {
    return {};
}