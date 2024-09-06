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
    explicit Impl() = default;
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

    /// CRUD
    StereoIndex addCloud(const PointCloud& item, RenderColor color,
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

        stereoProps_[stereoPropsIndexCount_] = actor;
        return stereoPropsIndexCount_++;
    }

    FlatIndex addText(Translation position, RenderColor color, int fontSize,
        const std::string& data) {
        auto [x, y, z] = position;
        auto [r, g, b] = color;

        auto textActor = vtkNew<vtkTextActor> {};
        textActor->SetInput(data.c_str());
        textActor->SetPosition2(x, y);
        auto property = textActor->GetTextProperty();
        property->SetFontSize(fontSize);
        property->SetColor(r, g, b);

        renderer_->AddActor2D(textActor);
        window_->Render();

        flatProps_[flatPropsIndexCount_] = textActor;
        return flatPropsIndexCount_++;
    }

    StereoIndex addPoint(Translation point, RenderColor color, double size, double alpha = 1.0) {
        auto [r, g, b] = color;
        auto [x, y, z] = point;

        vtkNew<vtkPoints> points;
        points->InsertNextPoint(x, y, z);

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

        stereoProps_[stereoPropsIndexCount_] = actor;
        return stereoPropsIndexCount_++;
    }

    StereoIndex addLine(Translation p1, Translation p2, RenderColor color, double width,
        double alpha = 1.0) {
        auto [r, g, b] = color;
        auto [x1, y1, z1] = p1;
        auto [x2, y2, z2] = p2;

        vtkNew<vtkPoints> points;
        points->InsertNextPoint(x1, y1, z1);
        points->InsertNextPoint(x2, y2, z2);

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

        stereoProps_[stereoPropsIndexCount_] = actor;
        return stereoPropsIndexCount_++;
    }

    StereoIndex addCoordinateSystem(Translation center, double length,
        double width, double alpha = 1.0) {
        auto [x, y, z] = center;
        vtkNew<vtkAxesActor> axesActor;
        axesActor->SetPosition(x, y, z);
        axesActor->SetTotalLength(length, length, length);
        axesActor->SetShaftType(vtkAxesActor::LINE_SHAFT);
        axesActor->SetConeRadius(width / 2.0);
        axesActor->AxisLabelsOff();

        renderer_->AddActor(axesActor);
        window_->Render();

        stereoProps_[stereoPropsIndexCount_] = axesActor;
        return stereoPropsIndexCount_++;
    }

    void removeStereoProps(StereoIndex index) {
        if (!stereoProps_.contains(index))
            return;
        renderer_->RemoveActor(stereoProps_[index]);
        stereoProps_.erase(index);
        window_->Render();
    }

    void removeFlatProps(FlatIndex index) {
        if (!flatProps_.contains(index))
            return;
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
        if (!stereoProps_.contains(index))
            return;
        stereoProps_[index]->SetVisibility(flag);
        window_->Render();
    }

    void setFlatPropsVisible(FlatIndex index, bool flag) {
        if (!flatProps_.contains(index))
            return;
        flatProps_[index]->SetVisibility(flag);
        window_->Render();
    }

    void modifyColor(StereoIndex index, double r, double g, double b) {
        if (!stereoProps_.contains(index))
            return;

        auto actor = dynamic_cast<vtkActor*>(stereoProps_[index].Get());
        actor->GetProperty()->SetColor(r, g, b);

        window_->Render();
    }

    void modifyVisible(StereoIndex index, bool flag) {
        if (!stereoProps_.contains(index))
            return;

        auto& actor = stereoProps_[index];
        actor->SetVisibility(flag);

        window_->Render();
    }

    void modifyPointSize(StereoIndex index, double size) {
        if (!stereoProps_.contains(index))
            return;
        spdlog::info("modifyPointSize: {} {}", index.index, size);

        auto actor = dynamic_cast<vtkActor*>(stereoProps_[index].Get());
        actor->GetProperty()->SetPointSize(size);

        window_->Render();
    }

    void transformCloud(StereoIndex index, Eigen::Affine3d transform) {
        if (!stereoProps_.contains(index))
            return;

        auto& actor = stereoProps_[index];

        const auto translation = Eigen::Translation3d { transform.translation() };
        actor->SetPosition(translation.x(),
            translation.y(), translation.z());

        const auto quaternion = Eigen::Quaterniond { transform.rotation() };
        actor->RotateWXYZ(quaternion.w(),
            quaternion.x(), quaternion.y(), quaternion.z());

        window_->Render();
    }

private:
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> window_;

    StereoIndex stereoPropsIndexCount_;
    std::unordered_map<StereoIndex, vtkSmartPointer<vtkProp3D>> stereoProps_;

    FlatIndex flatPropsIndexCount_;
    std::unordered_map<FlatIndex, vtkSmartPointer<vtkProp>> flatProps_;
};
}