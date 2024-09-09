#pragma once
#include "core/share/cloud-box.hh"
#include "core/share/object.hh"
#include "utility/single.hh"

#include <QVTKOpenGLNativeWidget.h>
#include <vtkAxesActor.h>
#include <vtkTextActor.h>

namespace core::renderer {
class Renderer : public util::Singleton<Renderer> {
public:
    Renderer(util::Singleton<Renderer>::token);
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    ~Renderer();

    /// Configuration
    void connectWidget(QVTKOpenGLNativeWidget* interface);

    void resetCamera();
    void render();

    template <vtkPropHandler Handler>
    void removeObject(Object<Handler>& object);

    static constexpr auto white = RenderColor { 1.0, 1.0, 1.0 };
    static constexpr auto black = RenderColor { 0.0, 0.0, 0.0 };
    static constexpr auto miku = RenderColor { 0.2235294117647059, 0.7725490196078432, 0.7333333333333333 };

    std::unique_ptr<CloudObject> makeCloud(const CloudBox& item, RenderColor color = white,
        float pointSize = 2.0);

    std::unique_ptr<PointObject> makePoint(const Eigen::Vector3d& point,
        const RenderColor& color = white, float size = 1.0);

    std::unique_ptr<TextObject> makeText(const Eigen::Vector3d& position, const std::string& data,
        RenderColor color = white, int fontSize = 16);

    std::unique_ptr<LineObject> makeLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
        const RenderColor& color, float width);

    std::unique_ptr<CoordinateObject> makeCoordinate(const Eigen::Vector3d& center,
        double length = 10.0, double width = 0.5);

    std::unique_ptr<CubeObject> makeCube(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
        const RenderColor& color, double alpha = 1.0);

private:
    struct Impl;
    Impl* pimpl_;
};

};

using Renderer = core::renderer::Renderer;