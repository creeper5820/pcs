#pragma once

#pragma once
#include "utility/single.hh"

#include <QVTKOpenGLNativeWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace core::cloud {

class Cloud : public util::Singleton<Cloud> {
public:
    Cloud(util::Singleton<Cloud>::token);
    ~Cloud();
    Cloud(const Cloud&) = delete;
    Cloud& operator=(const Cloud&) = delete;

    /// Configuration
    void connectUI(QVTKOpenGLNativeWidget* interface);

    /// CRUD
    bool loadCloud(const std::string& name);
    void removeCloud(const std::string& name);
    void saveCloud(const std::string& name, const std::string& path);
    void clearCloud();

    /// Modify property
    void modifyColor(const std::string& name, double r, double g, double b);
    void modifyPointSize(const std::string& name);
    void modifyVisible(const std::string& name, bool flag);

    void transformCloud(const std::string& name, Eigen::Affine3d transform);

    /// Select
    void addSelectCubeArea(int index, Eigen::Vector3d corner[2]);
    void addSelectCubeArea(int index, Eigen::Vector3d center, double length);
    void addSelectSphereArea(int index, Eigen::Vector3d center, double radius);

    void removeSelectArea(int index);
    void removeAllSelectArea();

    /// Operators after selecting
    void removeSelectCloud();
    void removeSelectCloud(const std::string& name);

    void segmentSelectCloud();
    void segmentSelectCloud(const std::string& name);

    void extractSelectCloud();
    void extractSelectCloud(const std::string& name);

private:
    struct Impl;
    Impl* pimpl_;
};
}

namespace core {
using Cloud = core::cloud::Cloud;
}