#pragma once
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

    /// CRUD
    bool loadCloud(int index, const std::string& path);
    void saveCloud(int index, const std::string& path);
    void removeCloud(int index);
    void removeAllCloud();

    /// Modify property
    void modifyColor(int index, double r, double g, double b);
    void modifyPointSize(int index, double size);
    void modifyVisible(int index, bool flag);

    void transformCloud(int index, Eigen::Affine3d transform);

    /// Select
    void addSelectCubeArea(int index, Eigen::Vector3d corner[2]);
    void addSelectCubeArea(int index, Eigen::Vector3d center, double length);
    void addSelectSphereArea(int index, Eigen::Vector3d center, double radius);

    void removeSelectArea(int index);
    void removeAllSelectArea();

    /// Operators after selecting
    void removeSelectCloud();
    void removeSelectCloud(int index);

    void segmentSelectCloud();
    void segmentSelectCloud(int index);

    void extractSelectCloud();
    void extractSelectCloud(int index);

private:
    struct Impl;
    Impl* pimpl_;
};
}

namespace core {
using Cloud = cloud::Cloud;
}