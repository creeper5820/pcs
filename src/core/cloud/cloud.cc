#include "core/cloud/cloud.hh"
#include <spdlog/spdlog.h>

using namespace core::cloud;
namespace speed = spdlog;

struct Cloud::Impl {
public:
private:
};

Cloud::Cloud(util::Singleton<Cloud>::token)
    : pimpl_(new Impl) {
    speed::info("Initialization of core::Cloud starts");
}

Cloud::~Cloud() {
    delete pimpl_;
    speed::info("core::Cloud exits");
}

/// Configuration
void Cloud::connectUI(QVTKOpenGLNativeWidget* interface) { }

/// CRUD
bool Cloud::loadCloud(const std::string& name) { }
void Cloud::removeCloud(const std::string& name) { }
void Cloud::saveCloud(const std::string& name, const std::string& path) { }
void Cloud::clearCloud() { }

/// Modify property
void Cloud::modifyColor(const std::string& name, double r, double g, double b) { }
void Cloud::modifyPointSize(const std::string& name) { }
void Cloud::modifyVisible(const std::string& name, bool flag) { }

void Cloud::transformCloud(const std::string& name, Eigen::Affine3d transform) { }

/// Select
void Cloud::addSelectCubeArea(int index, Eigen::Vector3d corner[2]) { }
void Cloud::addSelectCubeArea(int index, Eigen::Vector3d center, double length) { }
void Cloud::addSelectSphereArea(int index, Eigen::Vector3d center, double radius) { }

void Cloud::removeSelectArea(int index) { }
void Cloud::removeAllSelectArea() { }

/// Operators after selecting
void Cloud::removeSelectCloud() { }
void Cloud::removeSelectCloud(const std::string& name) { }

void Cloud::segmentSelectCloud() { }
void Cloud::segmentSelectCloud(const std::string& name) { }

void Cloud::extractSelectCloud() { }
void Cloud::extractSelectCloud(const std::string& name) { }