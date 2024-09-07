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
    explicit Impl() {
        objectKiller = [this](vtkProp* prop) {
            renderer_->RemoveActor(prop);
            window_->Render();
        };
    }
    ~Impl() = default;

    void connectWidget(QVTKOpenGLNativeWidget* interface) {
        renderer_ = vtkNew<vtkRenderer> {};
        window_ = vtkNew<vtkGenericOpenGLRenderWindow> {};

        interface->setRenderWindow(window_.Get());
        spdlog::info("connect widget with renderer");

        window_->SetWindowName("core::Renderer");
        window_->AddRenderer(renderer_);
        window_->Render();
    }

    void refreshCamera() {
        renderer_->ResetCamera();
        window_->Render();
    }

    void render() {
        window_->Render();
    }

    template <vtkPropHandler Handler>
    void removeObject(Object<Handler>& object) {
        renderer_->RemoveActor(object.handler());
        window_->Render();
    }

    /// CRUD
    std::unique_ptr<CloudObject> addCloud(const CloudBox& item, RenderColor color,
        double pointSize, double alpha) {
        vtkNew<vtkPoints> points;
        for (const auto point : item.points())
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

    FlatIndex addText(Eigen::Vector3d position, RenderColor color, int fontSize,
        const std::string& data) {
        auto [r, g, b] = color;

        auto textActor = vtkNew<vtkTextActor> {};
        textActor->SetInput(data.c_str());
        textActor->SetPosition2(position.x(), position.y());
        auto property = textActor->GetTextProperty();
        property->SetFontSize(fontSize);
        property->SetColor(r, g, b);

        renderer_->AddActor2D(textActor);
        window_->Render();

        flatProps_[flatPropsIndex_] = textActor;
        return flatPropsIndex_++;
    }

    std::unique_ptr<PointObject> addPoint(Eigen::Vector3d point,
        RenderColor color, double size, double alpha = 1.0) {
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
        actor->GetProperty()->SetOpacity(alpha);
        actor->GetProperty()->SetColor(r, g, b);
        actor->GetProperty()->SetPointSize(size);

        renderer_->AddActor(actor);
        window_->Render();

        return std::make_unique<PointObject>(actor);
    }

    StereoIndex addLine(Eigen::Vector3d p1, Eigen::Vector3d p2, RenderColor color, double width,
        double alpha = 1.0) {
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
        actor->GetProperty()->SetOpacity(alpha);
        actor->GetProperty()->SetColor(r, g, b);
        actor->GetProperty()->SetLineWidth(width);

        renderer_->AddActor(actor);
        window_->Render();

        stereoProps_[stereoPropsIndex_] = actor;
        return stereoPropsIndex_++;
    }

    StereoIndex addCoordinateSystem(Eigen::Vector3d center, double length,
        double width, double alpha = 1.0) {
        vtkNew<vtkAxesActor> axesActor;
        axesActor->SetPosition(center.x(), center.y(), center.z());
        axesActor->SetTotalLength(length, length, length);
        axesActor->SetShaftType(vtkAxesActor::LINE_SHAFT);
        axesActor->SetConeRadius(width / 2.0);
        axesActor->AxisLabelsOff();

        renderer_->AddActor(axesActor);
        window_->Render();

        stereoProps_[stereoPropsIndex_] = axesActor;
        return stereoPropsIndex_++;
    }

    void removeStereoProps(StereoIndex index) {
        renderer_->RemoveActor(stereoProps_[index]);
        stereoProps_.erase(index);
        window_->Render();
    }

    void removeFlatProps(FlatIndex index) {
        renderer_->RemoveActor(flatProps_[index]);
        flatProps_.erase(index);
        window_->Render();
    }

    void removeAllProps() {
        renderer_->RemoveAllViewProps();
        window_->Render();
    }

    /// Modify property
    void setStereoPropsVisible(StereoIndex index, bool flag) {
        stereoProps_[index]->SetVisibility(flag);
        window_->Render();
    }

    void setFlatPropsVisible(FlatIndex index, bool flag) {
        flatProps_[index]->SetVisibility(flag);
        window_->Render();
    }

    void modifyColor(StereoIndex index, double r, double g, double b) {
        auto actor = dynamic_cast<vtkActor*>(stereoProps_[index].Get());
        actor->GetProperty()->SetColor(r, g, b);

        window_->Render();
    }

    void modifyVisible(StereoIndex index, bool flag) {
        auto& actor = stereoProps_[index];
        actor->SetVisibility(flag);

        window_->Render();
    }

    void modifyPointSize(StereoIndex index, double size) {
        auto actor = dynamic_cast<vtkActor*>(stereoProps_[index].Get());
        actor->GetProperty()->SetPointSize(size);

        window_->Render();
    }

    void transformCloud(StereoIndex index, Eigen::Affine3d transform) {
        auto& actor = stereoProps_[index];

        const auto quaternion = Eigen::Quaterniond { transform.rotation() };
        actor->RotateWXYZ(quaternion.w(), quaternion.x(), quaternion.y(),
            quaternion.z());
        const auto translation = Eigen::Translation3d { transform.translation() };
        actor->SetPosition(translation.x(), translation.y(), translation.z());

        window_->Render();
    }

private:
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> window_;

    StereoIndex stereoPropsIndex_;
    std::unordered_map<StereoIndex, vtkSmartPointer<vtkProp3D>> stereoProps_;

    FlatIndex flatPropsIndex_;
    std::unordered_map<FlatIndex, vtkSmartPointer<vtkProp>> flatProps_;
};
}