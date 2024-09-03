#pragma once
#include "core/cloud/item.hh"
#include "utility/single.hh"

#include <QVTKOpenGLNativeWidget.h>

namespace core::renderer {

class Renderer : public util::Singleton<Renderer> {
public:
    using PointCloud = core::cloud::Item;

    Renderer(util::Singleton<Renderer>::token);
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    ~Renderer();

    /// Configuration
    void connectWidget(QVTKOpenGLNativeWidget* interface);

    /// CRUD
    void addCloud(int index, const PointCloud& item);
    void removeCloud(int index);
    void removeAllCloud();

    /// Modify property
    void modifyVisible(int index, bool flag);
    void modifyPointSize(int index, double size);
    void modifyColor(int index, double r, double g, double b);

    void transformCloud(int index);

private:
    struct Impl;
    Impl* pimpl_;
};

};

namespace core {
using Renderer = renderer::Renderer;
}
