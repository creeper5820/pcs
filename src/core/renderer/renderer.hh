#pragma once
#include "core/share/cloud-box.hh"
#include "core/share/index.hh"
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

    void refresh();
    void render();

    void removeObject(Object<vtkSmartPointer<vtkActor>>& object);
    void removeObject(Object<vtkSmartPointer<vtkTextActor>>& object);
    void removeObject(Object<vtkSmartPointer<vtkActor2D>>& object);
    void removeObject(Object<vtkSmartPointer<vtkAxesActor>>& object);

    std::unique_ptr<CloudObject> addCloud(const CloudBox& item, RenderColor color = { 1, 1, 1 },
        double pointSize = 2.0, double alpha = 1.0);
    std::unique_ptr<PointObject> addPoint(Eigen::Vector3d point, RenderColor color = { 1, 1, 1 },
        double size = 1.0, double alpha = 1.0);

    FlatIndex addText(const std::string& data, Eigen::Vector3d position,
        RenderColor color = { 1, 1, 1 }, int fontSize = 16);

    StereoIndex addLine(Eigen::Vector3d p1, Eigen::Vector3d p2, RenderColor color,
        double width, double alpha = 1.0);
    StereoIndex addCoordinateSystem(Eigen::Vector3d center, double length,
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

using Renderer = core::renderer::Renderer;