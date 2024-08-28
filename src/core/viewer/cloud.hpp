#pragma once
#include "item.hpp"

#include <QVTKOpenGLNativeWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>

namespace core::view {

class CloudView {
public:
    CloudView();
    ~CloudView();
    CloudView(const CloudView&) = delete;
    CloudView& operator=(const CloudView&) = delete;

    void bind_viewer(QVTKOpenGLNativeWidget* interface);
    void render();
    void reset_camera();

    [[nodiscard]] bool load_cloud(const std::string& path);
    void remove_cloud(const std::string& path);
    void clear_cloud();
    void clear();

    std::optional<std::reference_wrapper<Item>>
    operator[](const std::string& path);

    void set_color(const std::string& name,
        uint8_t r, uint8_t g, uint8_t b);

    // void set_size(const std::string& name);

    void add_coordinate_system(double scale = 1.0,
        const std::string& id = "reference", int viewport = 0);
    void remove_coordinate_system(const std::string& id);

private:
    struct Impl;
    Impl* pimpl_;
};

inline auto instance = CloudView {};
} // namespace core::view