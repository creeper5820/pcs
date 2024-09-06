#pragma once
#include "core/renderer/index.hh"
#include "utility/single.hh"

#include <QVTKOpenGLNativeWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace core::cloud {

class Cloud : public util::Singleton<Cloud> {
public:
    Cloud(util::Singleton<Cloud>::token);
    Cloud(const Cloud&) = delete;
    Cloud& operator=(const Cloud&) = delete;
    ~Cloud();

    /// Configuration
    void connectWidget(QVTKOpenGLNativeWidget* interface);

    void refresh();

    /// CRUD
    StereoIndex loadCloud(const std::string& path);
    void saveCloud(StereoIndex index, const std::string& path);
    void removeCloud(StereoIndex index);
    void removeAllCloud();

    /// Modify property
    void modifyColor(StereoIndex index, double r, double g, double b);
    void modifyPointSize(StereoIndex index, double size);
    void modifyVisible(StereoIndex index, bool flag);
    void transformCloud(StereoIndex index, Eigen::Affine3d transform);

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

namespace core {
using Cloud = cloud::Cloud;
}