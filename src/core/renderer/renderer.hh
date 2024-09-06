#pragma once
#include "core/cloud/item.hh"
#include "core/renderer/index.hh"
#include "utility/single.hh"

#include <QVTKOpenGLNativeWidget.h>

using PointCloud = core::cloud::Item;
using RenderColor = std::tuple<double, double, double>;
using Translation = std::tuple<double, double, double>;

namespace core::renderer {
class Renderer : public util::Singleton<Renderer> {
public:
    Renderer(util::Singleton<Renderer>::token);
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    ~Renderer();

    /// Configuration
    void connectWidget(QVTKOpenGLNativeWidget* interface);
    void refresh();

    StereoIndex addCloud(const PointCloud& item, RenderColor color,
        double pointSize, double alpha = 1.0);
    FlatIndex addText(Translation position, RenderColor color, int fontSize,
        const std::string& data);
    StereoIndex addPoint(Translation point, RenderColor color, double size,
        double alpha = 1.0);
    StereoIndex addLine(Translation p1, Translation p2, RenderColor color,
        double width, double alpha = 1.0);
    StereoIndex addCoordinateSystem(Translation center, double length,
        double width, double alpha = 1.0);

    void removeStereoProps(StereoIndex index);
    void removeFlatProps(FlatIndex index);
    void removeAllProps();

    /// Modify property
    void modifyVisible(StereoIndex index, bool flag);
    void modifyPointSize(StereoIndex index, double size);
    void modifyColor(StereoIndex index, double r, double g, double b);
    void transformCloud(StereoIndex index, Eigen::Affine3d transform);
    void setStereoPropsVisible(StereoIndex index, bool flag);
    void setFlatPropsVisible(FlatIndex index, bool flag);

private:
    struct Impl;
    Impl* pimpl_;
};

};

namespace core {
using Renderer = renderer::Renderer;
}
