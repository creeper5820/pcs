#pragma once
#include "item.hpp"
#include <QVTKOpenGLNativeWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>

namespace core::viewer {

class Storage {
public:
    Storage();
    ~Storage();
    Storage(const Storage&) = delete;
    Storage& operator=(const Storage&) = delete;

    void bind_viewer(QVTKOpenGLNativeWidget* interface);
    void refresh_viewer();

    [[nodiscard]] bool load_cloud(const std::string& path);
    void remove_cloud(const std::string& path);
    void clear_cloud();
    void clear();

    std::optional<std::reference_wrapper<Item>>
    operator[](const std::string& path);

    // void set_color(const std::string& name);
    // void set_size(const std::string& name);

    void set_coordinate_system(double scale = 1.0,
        const std::string& id = "reference", int viewport = 0);

private:
    struct Impl;
    Impl* pimpl_;
};

inline auto storage = Storage {};
} // namespace core::viewer