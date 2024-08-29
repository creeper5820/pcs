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
        util::bind_vtk(visualizer_, interface, "qt");
        visualizer_->getRenderWindow()->GlobalWarningDisplayOff();
        visualizer_render();
    }

    inline bool load_cloud(const std::string& path) {
        auto contains = true;
        if (!items_.contains(path)) {
            items_[path] = std::make_shared<Item>(path);
            contains = false;
        }

        auto& cloud = *items_[path];
        if (!cloud.loaded()) {
            if (!contains)
                items_.erase(path);
            util::message("load", "failed");
            return false;
        } else {
            cloud.set_color(255, 255, 255);
            visualizer_->addPointCloud(*cloud, path);
            util::message("load", "size",
                std::to_string(cloud.size()), "path", path);
            visualizer_render();
            return true;
        }
    }

    inline std::shared_ptr<Item> operator[](const std::string& path) {
        return items_.contains(path) ? items_[path] : nullptr;
    }

    inline void visualizer_render() {
        visualizer_->getRenderWindow()->Render();
    }
    inline void reset_camera() {
        visualizer_->resetCamera();
    }

    inline void remove_cloud(const std::string& name) {
        items_.erase(name);
        visualizer_->removePointCloud(name);
        visualizer_render();
        util::message("remove", name,
            "clouds", items_.size());
    }

    inline void clear_cloud() {
        visualizer_->removeAllPointClouds();
        items_.clear();
        visualizer_render();
        util::message("clear", "clouds");
    }

    inline void add_coordinate_system(double scale = 1.0,
        const std::string& id = "reference", int viewport = 0) {
        visualizer_->addCoordinateSystem(scale, id, viewport);
        visualizer_render();
    }
    inline void remove_coordinate_system(const std::string& id) {
        visualizer_->removeCoordinateSystem(id);
        visualizer_render();
    }

    inline void set_color(const std::string& name, uint8_t r, uint8_t g, uint8_t b) {
        if (!items_.contains(name)) {
            util::message("unknown cloud");
            return;
        }
        // ??? 你妈的不用引用 ???
        // 浪费我TM的两天时间找你这个BUG
        // 给我老老实实用智能指针
        auto item = items_[name];
        item->set_color(r, g, b);
        visualizer_->updatePointCloud(**item, name);
        visualizer_render();
    }

private:
    pcl::visualization::PCLVisualizer::Ptr visualizer_;

    std::map<std::string, std::shared_ptr<Item>> items_;

    bool enable_info_ { true };
};

CloudView::CloudView(typename util::Singleton<CloudView>::token) {
    pimpl_ = new Impl {};
}

CloudView::~CloudView() {
    delete pimpl_;
}

void CloudView::bind_viewer(QVTKOpenGLNativeWidget* interface) {
    pimpl_->register_viewer(interface);
}

bool CloudView::load_cloud(const std::string& path) {
    return pimpl_->load_cloud(path);
}

void CloudView::remove_cloud(const std::string& path) {
    pimpl_->remove_cloud(path);
}

void CloudView::clear_cloud() {
    pimpl_->clear_cloud();
}

void CloudView::clear() {
    pimpl_->clear_cloud();
}

void CloudView::render() {
    pimpl_->visualizer_render();
}

void CloudView::reset_camera() {
    pimpl_->reset_camera();
}

void CloudView::set_color(const std::string& name,
    uint8_t r, uint8_t g, uint8_t b) {
    pimpl_->set_color(name, r, g, b);
}

void CloudView::add_coordinate_system(
    double scale, const std::string& id, int viewport) {
    pimpl_->add_coordinate_system(scale, id, viewport);
}

void CloudView::remove_coordinate_system(
    const std::string& id) {
    pimpl_->remove_coordinate_system(id);
}

std::shared_ptr<Item>
CloudView::operator[](const std::string& path) {
    return (*pimpl_)[path];
}
