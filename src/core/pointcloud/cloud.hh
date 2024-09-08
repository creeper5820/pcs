#pragma once
#include "core/share/cloud-box.hh"
#include "core/share/object.hh"
#include "utility/single.hh"

#include <QVTKOpenGLNativeWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace core::cloud {

struct CloudPackage {
    std::unique_ptr<CloudSource> source;
    std::unique_ptr<CloudObject> object;
};

class CloudManager : public util::Singleton<CloudManager> {
public:
    CloudManager(util::Singleton<CloudManager>::token);
    CloudManager(const CloudManager&) = delete;
    CloudManager& operator=(const CloudManager&) = delete;
    ~CloudManager();

    /// Configuration
    void connectWidget(QVTKOpenGLNativeWidget* interface);

    void refresh();

    std::unique_ptr<CloudPackage> makePackage(const std::string& path);

    /// Select
    int addSelectPointArea(Eigen::Vector3d point);
    int addSelectCubeArea(Eigen::Vector3d corner[2]);
    int addSelectCubeArea(Eigen::Vector3d center, double length);
    int addSelectSphereArea(Eigen::Vector3d center, double radius);

    void removeAllSelectArea();
    void removeSelectArea(int index);

    /// Operators after selecting
    void removeAllSelectCloud();
    void removeSelectCloud(int index);

    int extractAllSelectCloud();
    int extractSelectCloud(int index);

private:
    struct Impl;
    Impl* pimpl_;
};
}

using CloudPackage = core::cloud::CloudPackage;
using CloudManager = core::cloud::CloudManager;