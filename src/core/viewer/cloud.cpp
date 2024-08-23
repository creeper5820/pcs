#include "cloud.hpp"
#include "utility/common.hpp"

#include <QMessageBox>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <memory>
#include <qchar.h>
#include <qmessagebox.h>
#include <string>

using namespace core::viewer;

struct Storage::Impl {
public:
    Impl()
    {
    }

    inline void register_viewer(QVTKOpenGLNativeWidget* interface)
    {
        utility::bind_vtk(visualizer_, interface, "qt");
    }

    inline void load_cloud(const std::string& path, const std::string& name)
    {
        auto contains = true;
        if (!clouds_.contains(name)) {
            clouds_[name] = std::make_shared<PointCloudT>();
            contains = false;
        }

        auto& cloud = clouds_[name];
        if (pcl::io::loadPCDFile(path, *cloud) == -1) {
            QMessageBox::information(nullptr, "info", "failed to load cloud");
            if (!contains)
                clouds_.erase(name);
        } else {
            auto msg = "load cloud successfully, size: " + std::to_string(cloud->size());
            QMessageBox::information(nullptr, "info", msg.data());
            visualizer_->addPointCloud(cloud, name);
            visualizer_->resetCamera();
        }
    }

private:
    pcl::visualization::PCLVisualizer::Ptr visualizer_;
    std::map<std::string, PointCloudT::Ptr> clouds_;

    bool enable_info_ { true };
};

Storage::Storage() { pimpl_ = new Impl {}; }
Storage::~Storage() { delete pimpl_; }

void Storage::bind_viewer(QVTKOpenGLNativeWidget* interface)
{
    pimpl_->register_viewer(interface);
}

void Storage::load_cloud(const std::string& path, const std::string& name)
{
    pimpl_->load_cloud(path, name);
}

void Storage::remove_cloud(const std::string& name) { }

void Storage::clear_cloud() { }

void Storage::clear() { }

void Storage::refresh_viewer() { }