#pragma once
#include <QVTKOpenGLNativeWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace core::viewer {

class Storage {
public:
    Storage();
    ~Storage();
    Storage(const Storage&) = delete;
    Storage& operator=(const Storage&) = delete;

    using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

    // @brief bind the qt vtk window with pcl visualizer
    void bind_viewer(QVTKOpenGLNativeWidget* interface);

    // @brief as you can see, load a point cloud
    bool load_cloud(const std::string& path, const std::string& name);

    // @brief as you can see, remove a point cloud
    void remove_cloud(const std::string& name);

    // @brief clear all the point clouds
    void clear_cloud();

    // TODO: clear ...

    // @brief clear all
    void clear();

    void refresh_viewer();

private:
    struct Impl;
    Impl* pimpl_;
};

inline auto storage = Storage {};
}