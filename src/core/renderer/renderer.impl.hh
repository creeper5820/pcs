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
    int addCloud(const PointCloud& item) {
        static int cloudIndex = -1;
        cloudIndex++;

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
        cloudActors_[cloudIndex] = actor;

        window_->Render();

        return cloudIndex;
    }

    void removeCloud(int index) {
        if (!cloudActors_.contains(index))
            return;

        renderer_->RemoveActor(cloudActors_[index]);
        cloudActors_.erase(index);

        window_->Render();
    }

    void removeAllCloud() {
        for (auto [index, actor] : cloudActors_)
            renderer_->RemoveActor(actor);
        cloudActors_.clear();

        window_->Render();
    }

    /// Modify property
    void modifyColor(int index, double r, double g, double b) {
        if (!cloudActors_.contains(index))
            return;

        auto& actor = cloudActors_[index];
        actor->GetProperty()->SetColor(r, g, b);

        window_->Render();
    }

    void modifyVisible(int index, bool flag) {
        if (!cloudActors_.contains(index))
            return;

        auto& actor = cloudActors_[index];
        actor->SetVisibility(flag);

        window_->Render();
    }

    void modifyPointSize(int index, double size) {
        if (!cloudActors_.contains(index))
            return;

        auto& actor = cloudActors_[index];
        actor->GetProperty()->SetPointSize(size);

        window_->Render();
    }

    void transformCloud(int index, Eigen::Affine3d transform) {
        if (!cloudActors_.contains(index))
            return;

        auto& actor = cloudActors_[index];

        const auto translation = Eigen::Translation3d { transform.translation() };
        actor->SetPosition(translation.x(),
            translation.y(), translation.z());

        const auto quaternion = Eigen::Quaterniond { transform.rotation() };
        actor->RotateWXYZ(quaternion.w(),
            quaternion.x(), quaternion.y(), quaternion.z());

        window_->Render();
    }

    /// Visualization
    int addText(const TextConfig& text) {
        auto textActor = vtkNew<vtkTextActor> {};
        textActor->SetInput(text.data.c_str());
        textActor->SetPosition2(text.position.x(), text.position.y());

        auto property = textActor->GetTextProperty();
        property->SetFontSize(text.fontSize);
        auto [r, g, b] = text.color;
        property->SetColor(r, g, b);

        renderer_->AddActor2D(textActor);
        window_->Render();

        flatProps_[flatPropsIndexCount_] = textActor;
        return flatPropsIndexCount_++;
    }

    int addPoint(Eigen::Vector3d point, RGB color, double size, double alpha = 1.0) {
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

        stereoProps_[stereoPropsIndexCount_] = actor;
        return stereoPropsIndexCount_++;
    }

    int addLine(Eigen::Vector3d p1, Eigen::Vector3d p2, RGB color, double width,
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

        stereoProps_[stereoPropsIndexCount_] = actor;
        return stereoPropsIndexCount_++;
    }

    int addCoordinateSystem(Eigen::Vector3d center, double length,
        double width, double alpha = 1.0) {
        vtkNew<vtkAxesActor> axesActor;
        axesActor->SetPosition(center.x(), center.y(), center.z());
        axesActor->SetTotalLength(length, length, length);
        axesActor->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);
        axesActor->SetCylinderRadius(width);
        axesActor->SetConeRadius(width);
        axesActor->SetSphereRadius(width);

        renderer_->AddActor(axesActor);
        window_->Render();

        stereoProps_[stereoPropsIndexCount_] = axesActor;
        return stereoPropsIndexCount_++;
    }

    void removeStereoProps(int index) {
        if (!stereoProps_.contains(index))
            return;
        renderer_->RemoveActor(stereoProps_[index]);
        stereoProps_.erase(index);
        window_->Render();
    }

    void removeFlatProps(int index) {
        if (!flatProps_.contains(index))
            return;
        renderer_->RemoveActor(flatProps_[index]);
        flatProps_.erase(index);
        window_->Render();
    }

    void setStereoPropsVisible(int index, bool flag) {
        if (!stereoProps_.contains(index))
            return;
        stereoProps_[index]->SetVisibility(flag);
        window_->Render();
    }

    void setFlatPropsVisible(int index, bool flag) {
        if (!flatProps_.contains(index))
            return;
        flatProps_[index]->SetVisibility(flag);
        window_->Render();
    }

private:
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> window_;

    int cloudIndexCount_ = 0;
    std::unordered_map<int, vtkSmartPointer<vtkActor>> cloudActors_;

    int stereoPropsIndexCount_ = 0;
    std::unordered_map<int, vtkSmartPointer<vtkProp3D>> stereoProps_;

    int flatPropsIndexCount_ = 0;
    std::unordered_map<int, vtkSmartPointer<vtkProp>> flatProps_;
};
}