#pragma once
#include "core/renderer/renderer.hh"
#include "core/share/object.hh"
#include "core/share/objects.hh"

#include <vtkAbstractPicker.h>
#include <vtkAxesActor.h>
#include <vtkBuffer.h>
#include <vtkCubeSource.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRendererCollection.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkVertexGlyphFilter.h>

#include <spdlog/spdlog.h>

namespace core::renderer {
struct Renderer::Impl {
public:
    void connectWidget(QVTKOpenGLNativeWidget* interface) {
        renderer_ = vtkNew<vtkRenderer> {};
        window_ = vtkNew<vtkGenericOpenGLRenderWindow> {};

        // Be sure that the widget is registered before the renderer is set
        interface->setRenderWindow(window_);

        window_->SetWindowName("core::Renderer");
        window_->AddRenderer(renderer_);
        window_->Render();

        objectKiller = [this](vtkProp* prop) {
            renderer_->RemoveActor(prop);
            window_->Render();
        };

        spdlog::info("connect widget with renderer");
    }

    void resetCamera() {
        renderer_->ResetCamera();
    }

    void render() {
        window_->Render();
    }

    template <vtkPropHandler Handler>
    void removeObject(Object<Handler>& object) {
        renderer_->RemoveActor(object.handler());
        window_->Render();
    }

    /// Objects
    std::unique_ptr<CloudObject> makeCloud(const CloudSource& cloud,
        const RenderColor& color, float pointSize) {
        vtkNew<vtkPoints> points;
        for (const auto point : cloud.points())
            points->InsertNextPoint(point.x, point.y, point.z);

        vtkNew<vtkPolyData> polyData;
        polyData->SetPoints(points);

        vtkNew<vtkVertexGlyphFilter> vertexFilter;
        vertexFilter->SetInputData(polyData);
        vertexFilter->Update();
        polyData->ShallowCopy(vertexFilter->GetOutput());

        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputData(polyData);

        vtkNew<vtkActor> actor;
        actor->SetMapper(mapper);
        actor->GetProperty()->SetPointSize(pointSize);

        renderer_->AddActor(actor);
        window_->Render();

        return std::make_unique<CloudObject>(actor);
    }

    std::unique_ptr<PointObject> makePoint(const Eigen::Vector3d& point,
        const RenderColor& color, float size) {
        auto [r, g, b] = color;

        vtkNew<vtkPoints> points;
        points->InsertNextPoint(point.x(), point.y(), point.z());

        vtkNew<vtkPolyData> polyData;
        polyData->SetPoints(points);

        vtkNew<vtkVertexGlyphFilter> vertexFilter;
        vertexFilter->SetInputData(polyData);
        vertexFilter->Update();
        polyData->ShallowCopy(vertexFilter->GetOutput());

        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputData(polyData);

        vtkNew<vtkActor> actor;
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(r, g, b);
        actor->GetProperty()->SetPointSize(size);

        renderer_->AddActor(actor);
        window_->Render();

        return std::make_unique<PointObject>(actor);
    }

    std::unique_ptr<LineObject> makeLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
        const RenderColor& color, float width) {
        auto [r, g, b] = color;

        vtkNew<vtkPoints> points;
        points->InsertNextPoint(p1.x(), p1.y(), p1.z());
        points->InsertNextPoint(p2.x(), p2.y(), p2.z());

        vtkNew<vtkPolyData> polyData;
        polyData->SetPoints(points);

        vtkNew<vtkCellArray> lines;
        lines->InsertNextCell(2);
        lines->InsertCellPoint(0);
        lines->InsertCellPoint(1);
        polyData->SetLines(lines);

        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputData(polyData);

        vtkNew<vtkActor> actor;
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(r, g, b);
        actor->GetProperty()->SetLineWidth(width);

        renderer_->AddActor(actor);
        window_->Render();

        return std::make_unique<LineObject>(actor);
    }

    std::unique_ptr<PlaneObject> makePlane(
        const Eigen::Vector3d& p0,
        const Eigen::Vector3d& p1,
        const Eigen::Vector3d& p2) {

        vtkNew<vtkPlaneSource> planeSource;
        planeSource->SetOrigin(p0.x(), p0.y(), p0.z());
        planeSource->SetPoint1(p1.x(), p1.y(), p1.z());
        planeSource->SetPoint2(p2.x(), p2.y(), p2.z());
        planeSource->Update();

        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputData(planeSource->GetOutput());

        vtkNew<vtkActor> actor;
        actor->SetMapper(mapper);

        renderer_->AddActor(actor);
        window_->Render();

        return std::make_unique<PlaneObject>(actor);
    }

    std::unique_ptr<CubeObject> makeCube(
        const Eigen::Vector3d& p1,
        const Eigen::Vector3d& p2,
        const RenderColor& color,
        double alpha) {

        auto [r, g, b] = color;

        const auto center = (p1 + p2) / 2;
        const auto xLength = std::abs(p1.x() - p2.x());
        const auto yLength = std::abs(p1.y() - p2.y());
        const auto zLength = std::abs(p1.z() - p2.z());

        vtkNew<vtkCubeSource> cubeSource;
        cubeSource->SetCenter(center.x(), center.y(), center.z());
        cubeSource->SetXLength(xLength);
        cubeSource->SetYLength(yLength);
        cubeSource->SetZLength(zLength);
        cubeSource->Update();

        vtkNew<vtkPolyDataMapper> cubePolyDataMapper;
        cubePolyDataMapper->SetInputData(cubeSource->GetOutput());

        vtkNew<vtkActor> cubeActor;
        cubeActor->SetMapper(cubePolyDataMapper);
        cubeActor->GetProperty()->SetColor(r, g, b);
        cubeActor->GetProperty()->SetOpacity(alpha);

        renderer_->AddActor(cubeActor);
        window_->Render();

        return std::make_unique<CubeObject>(cubeActor);
    }

    std::unique_ptr<TextObject> makeText(const Eigen::Vector3d& position, int fontSize,
        const RenderColor& color, const std::string& data) {
        auto [r, g, b] = color;

        auto textActor = vtkNew<vtkTextActor> {};
        textActor->SetInput(data.c_str());
        textActor->SetPosition2(position.x(), position.y());
        auto property = textActor->GetTextProperty();
        property->SetFontSize(fontSize);
        property->SetColor(r, g, b);

        renderer_->AddActor2D(textActor);
        window_->Render();

        return std::make_unique<TextObject>(textActor);
    }

    std::unique_ptr<CoordinateObject> makeCoordinate(const Eigen::Vector3d& center,
        double length, double width) {
        vtkNew<vtkAxesActor> axesActor;
        axesActor->SetPosition(center.x(), center.y(), center.z());
        axesActor->SetTotalLength(length, length, length);
        axesActor->SetShaftType(vtkAxesActor::LINE_SHAFT);
        axesActor->SetConeRadius(width / 2.0);
        axesActor->AxisLabelsOff();

        renderer_->AddActor(axesActor);
        window_->Render();

        return std::make_unique<CoordinateObject>(axesActor);
    }

    // Select Helper
    std::unique_ptr<CubeObjects> makeCloudSelectCube(const CloudSource& cloud,
        const RenderColor& color, double resolution) {
        auto cubeObjects = std::make_unique<CubeObjects>();
        auto points = cloud.points();

        return cubeObjects;
    }

private:
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> window_;
};
}