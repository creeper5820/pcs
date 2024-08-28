#include "cloud.hpp"
#include "item.hpp"
#include "utility/common.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace core::view;

struct CloudView::Impl {
public:
    Impl() { }

    inline void register_viewer(QVTKOpenGLNativeWidget* interface) {
        utility::bind_vtk(visualizer_, interface, "qt");
        render();
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
            render();
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

    inline void render() {
        visualizer_->getRenderWindow()->Render();
    }
    inline void reset_camera() {
        visualizer_->resetCamera();
    }

    inline void remove_cloud(const std::string& name) {
        clouds_.erase(name);
        visualizer_->removePointCloud(name);
        render();
        utility::message("remove", name, "clouds", clouds_.size());
    }

    inline void clear_cloud() {
        visualizer_->removeAllPointClouds();
        clouds_.clear();
        render();
        utility::message("clear", "clouds");
    }

    inline void add_coordinate_system(double scale = 1.0,
        const std::string& id = "reference", int viewport = 0) {
        visualizer_->addCoordinateSystem(scale, id, viewport);
        render();
    }
    inline void remove_coordinate_system(const std::string& id) {
        visualizer_->removeCoordinateSystem(id);
        render();
    }

    inline void set_color(const std::string& name, uint8_t r, uint8_t g, uint8_t b) {
        if (!clouds_.contains(name)) {
            utility::message("unknown cloud");
            return;
        }

        auto item = *clouds_[name];
        item.set_color(r, g, b);
        visualizer_->updatePointCloud(*item, name);
        render();
    }

private:
    pcl::visualization::PCLVisualizer::Ptr visualizer_;

    std::map<std::string, std::unique_ptr<Item>> clouds_;

    bool enable_info_ { true };
};

CloudView::CloudView() { pimpl_ = new Impl {}; }

CloudView::~CloudView() { delete pimpl_; }

void CloudView::bind_viewer(QVTKOpenGLNativeWidget* interface) {
    pimpl_->register_viewer(interface);
}

bool CloudView::load_cloud(const std::string& path) {
    return pimpl_->load_cloud(path);
}

void CloudView::remove_cloud(const std::string& path) {
    pimpl_->remove_cloud(path);
}

void CloudView::clear_cloud() { pimpl_->clear_cloud(); }

void CloudView::clear() { pimpl_->clear_cloud(); }

void CloudView::render() { pimpl_->render(); }

void CloudView::reset_camera() { pimpl_->reset_camera(); }

void CloudView::set_color(const std::string& name,
    uint8_t r, uint8_t g, uint8_t b) {
    pimpl_->set_color(name, r, g, b);
}

void CloudView::add_coordinate_system(
    double scale, const std::string& id, int viewport) {
    pimpl_->add_coordinate_system(scale, id, viewport);
}

void CloudView::remove_coordinate_system(const std::string& id) {
    pimpl_->remove_coordinate_system(id);
}

std::optional<std::reference_wrapper<Item>>
CloudView::operator[](const std::string& path) {
    return (*pimpl_)[path];
}
