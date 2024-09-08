#pragma once
#include "core/renderer/renderer.hh"

#include <vtkAbstractPicker.h>
#include <vtkAxesActor.h>
#include <vtkBuffer.h>
#include <vtkGenericOpenGLRenderWindow.h>
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
    std::unique_ptr<CloudObject> makeCloud(const CloudBox& cloudBox,
        const RenderColor& color, double pointSize) {
        vtkNew<vtkPoints> points;
        for (const auto point : cloudBox.points())
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

        renderer_->AddActor(actor);
        window_->Render();

        return std::make_unique<CloudObject>(actor);
    }

    std::unique_ptr<PointObject> makePoint(const Eigen::Vector3d& point,
        const RenderColor& color, double size) {

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

    std::unique_ptr<LineObject> makeLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
        const RenderColor& color, double width) {
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

    std::unique_ptr<CoordinateObject> makeCoordinate(const Eigen::Vector3d& center, double length,
        double width, double alpha = 1.0) {
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

private:
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> window_;
};
}