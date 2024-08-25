#include "cloud.hpp"
#include "item.hpp"
#include "utility/common.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace core::viewer;

struct Storage::Impl {
public:
    Impl() { }

    inline void register_viewer(QVTKOpenGLNativeWidget* interface) {
        utility::bind_vtk(visualizer_, interface, "qt");
        refresh_viewer();
        visualizer_->getRenderWindow()->GlobalWarningDisplayOff();
    }

    inline bool load_cloud(const std::string& path) {
        auto contains = true;
        if (!clouds_.contains(path)) {
            clouds_[path] = std::make_unique<Item>(path);
            contains = false;
        }

        auto& cloud = *clouds_[path];
        if (!cloud.loaded()) {
            if (!contains)
                clouds_.erase(path);
            utility::message("load", "failed");
            return false;
        } else {
            visualizer_->addPointCloud(*cloud, path);
            utility::message("load", "size",
                std::to_string(cloud.size()), "path", path);
            refresh_viewer();
            return true;
        }
    }

    inline std::optional<std::reference_wrapper<Item>>
    operator[](const std::string& path) {
        if (clouds_.contains(path))
            return { *clouds_[path] };
        else
            return std::nullopt;
    }

    inline void refresh_viewer() {
        visualizer_->getRenderWindow()->Render();
        visualizer_->resetCamera();
    }

    inline void remove_cloud(const std::string& name) {
        clouds_.erase(name);
        visualizer_->removePointCloud(name);
        refresh_viewer();
        utility::message("remove", name, "clouds", clouds_.size());
    }

    inline void clear_cloud() {
        visualizer_->removeAllPointClouds();
        clouds_.clear();
        refresh_viewer();
        utility::message("clear", "clouds");
    }

    inline void set_coordinate_system(double scale = 1.0,
        const std::string& id = "reference", int viewport = 0) {
        visualizer_->addCoordinateSystem(scale, id, viewport);
    }

    inline void set_color(const std::string& name, pcl::RGB rgb) {
        if (!clouds_.contains(name))
            return;
        clouds_[name]->set_color(rgb);
    }

private:
    pcl::visualization::PCLVisualizer::Ptr visualizer_;

    std::map<std::string, std::unique_ptr<Item>> clouds_;

    bool enable_info_ { true };
};

Storage::Storage() { pimpl_ = new Impl {}; }
Storage::~Storage() { delete pimpl_; }

void Storage::bind_viewer(QVTKOpenGLNativeWidget* interface) {
    pimpl_->register_viewer(interface);
}

bool Storage::load_cloud(const std::string& path) {
    return pimpl_->load_cloud(path);
}

void Storage::remove_cloud(const std::string& path) {
    pimpl_->remove_cloud(path);
}

void Storage::clear_cloud() { pimpl_->clear_cloud(); }

void Storage::clear() { pimpl_->clear_cloud(); }

void Storage::refresh_viewer() { pimpl_->refresh_viewer(); }

void Storage::set_coordinate_system(
    double scale, const std::string& id, int viewport) {
    pimpl_->set_coordinate_system(scale, id, viewport);
}

std::optional<std::reference_wrapper<Item>>
Storage::operator[](const std::string& path) {
    return (*pimpl_)[path];
}
