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
        refresh_viewer();
    }

    inline bool load_cloud(const std::string& path, const std::string& name)
    {
        auto contains = true;
        if (!clouds_.contains(name)) {
            clouds_[name] = std::make_shared<PointCloudT>();
            contains = false;
        }

        auto& cloud = clouds_[name];
        if (pcl::io::loadPCDFile(path, *cloud) != 0) {
            if (!contains)
                clouds_.erase(name);
            utility::message("load", "failed");
            return false;
        } else {
            visualizer_->addPointCloud(cloud, name);
            utility::message("load", "size",
                std::to_string(cloud->size()), "path", path);
            refresh_viewer();
            return true;
        }
    }

    inline void refresh_viewer()
    {
        visualizer_->getRenderWindow()->Render();
        visualizer_->resetCamera();
    }

    inline void remove_cloud(const std::string& name)
    {
        clouds_.erase(name);
        visualizer_->removePointCloud(name);
        refresh_viewer();
        utility::message("remove", name, "clouds", clouds_.size());
    }

    inline void clear_cloud()
    {
        visualizer_->removeAllPointClouds();
        clouds_.clear();
        refresh_viewer();
        utility::message("clear", "clouds");
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

bool Storage::load_cloud(const std::string& path, const std::string& name)
{
    return pimpl_->load_cloud(path, name);
}

void Storage::remove_cloud(const std::string& name) { pimpl_->remove_cloud(name); }

void Storage::clear_cloud() { pimpl_->clear_cloud(); }

void Storage::clear() { }

void Storage::refresh_viewer() { pimpl_->refresh_viewer(); }