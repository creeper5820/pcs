#pragma once
#include "item.hpp"
#include "utility/single.hpp"

#include <QVTKOpenGLNativeWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace core::view {

/// @brief Point Cloud Manager and Visualizer
/// @note based on Mayer's singleton
class CloudView : public util::Singleton<CloudView> {
public:
    CloudView(typename util::Singleton<CloudView>::token);
    ~CloudView();
    CloudView(const CloudView&) = delete;
    CloudView& operator=(const CloudView&) = delete;

    void bind_viewer(QVTKOpenGLNativeWidget* interface);

    void reset_camera();
    void clear_cloud();
    void clear();
    void render();

    [[nodiscard]] bool load_cloud(const std::string& path);

    void remove_cloud(const std::string& path);

    void set_color(const std::string& name,
        uint8_t r, uint8_t g, uint8_t b);

    void add_coordinate_system(double scale = 1.0,
        const std::string& id = "reference", int viewport = 0);
    void remove_coordinate_system(const std::string& id);

    std::shared_ptr<Item> operator[](const std::string& path);

private:
    struct Impl;
    Impl* pimpl_;
};
}

namespace core {
/// @brief use instance like this: core::View::instance()
/// @return reference of the instance
using View = view::CloudView;
}