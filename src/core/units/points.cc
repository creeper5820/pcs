#include "points.hh"

#include "core/renderer.vtk.hh"

#include <vtk/vtkAxesActor.h>
#include <vtk/vtkPointPicker.h>
#include <vtk/vtkPolyData.h>
#include <vtk/vtkPolyDataMapper.h>
#include <vtk/vtkRenderer.h>
#include <vtk/vtkSphereSource.h>
#include <vtk/vtkTransform.h>
#include <vtk/vtkVertexGlyphFilter.h>

#include <algorithm>
#include <array>

namespace pcs {

struct PointsUnit::Impl {
    SmartPointer<vtkPoints> points;
    SmartPointer<vtkPolyData> data;
    SmartPointer<vtkPolyDataMapper> mapper;
    SmartPointer<vtkActor> actor;
    SmartPointer<vtkAxesActor> coordinate;
    SmartPointer<vtkTransform> coordinate_transform;
    SmartPointer<vtkSphereSource> coordinate_origin;
    SmartPointer<vtkPolyDataMapper> coordinate_origin_mapper;
    SmartPointer<vtkActor> coordinate_origin_actor;

    auto initialize(SmartPointer<vtkPoints> _points) {
        points = std::move(_points);

        data = vtkPolyData::New();
        data->SetPoints(points);

        using Filter = vtkVertexGlyphFilter;
        auto filter  = SmartPointer<Filter> { Filter::New() };
        filter->SetInputData(data);
        filter->Update();
        data->ShallowCopy(filter->GetOutput());

        mapper = vtkPolyDataMapper::New();
        mapper->SetInputData(data);

        actor = vtkActor::New();
        actor->SetMapper(mapper);

        coordinate = vtkAxesActor::New();
        coordinate->AxisLabelsOff();
        coordinate->PickableOff();
        coordinate->GetXAxisShaftProperty()->SetLineWidth(3.0);
        coordinate->GetYAxisShaftProperty()->SetLineWidth(3.0);
        coordinate->GetZAxisShaftProperty()->SetLineWidth(3.0);
        coordinate->GetXAxisTipProperty()->SetColor(1.0, 0.42, 0.32);
        coordinate->GetYAxisTipProperty()->SetColor(0.22, 0.76, 0.47);
        coordinate->GetZAxisTipProperty()->SetColor(0.32, 0.62, 1.0);

        auto bounds = std::array<double, 6> { };
        data->GetBounds(bounds.data());
        const auto x_size = std::max(0.0, bounds[1] - bounds[0]);
        const auto y_size = std::max(0.0, bounds[3] - bounds[2]);
        const auto z_size = std::max(0.0, bounds[5] - bounds[4]);
        const auto axis_length = std::max(0.2, std::max({ x_size, y_size, z_size }) * 0.1);
        coordinate->SetTotalLength(axis_length, axis_length, axis_length);

        coordinate_transform = vtkTransform::New();
        coordinate->SetUserTransform(coordinate_transform);

        coordinate_origin = vtkSphereSource::New();
        coordinate_origin->SetCenter(0.0, 0.0, 0.0);
        coordinate_origin->SetRadius(std::max(0.03, axis_length * 0.06));
        coordinate_origin->SetThetaResolution(24);
        coordinate_origin->SetPhiResolution(24);

        coordinate_origin_mapper = vtkPolyDataMapper::New();
        coordinate_origin_mapper->SetInputConnection(coordinate_origin->GetOutputPort());

        coordinate_origin_actor = vtkActor::New();
        coordinate_origin_actor->SetMapper(coordinate_origin_mapper);
        coordinate_origin_actor->PickableOff();
        coordinate_origin_actor->GetProperty()->LightingOff();
        coordinate_origin_actor->GetProperty()->SetColor(0.95, 0.95, 0.95);
        coordinate_origin_actor->GetProperty()->SetOpacity(0.9);

    }
};

auto PointsUnit::initialize(SmartPointer<vtkPoints> points) noexcept -> void {
    pimpl->initialize(points);
}

auto PointsUnit::actor() noexcept -> SmartPointer<vtkActor> { return pimpl->actor; }

auto PointsUnit::actor() const noexcept -> SmartPointer<vtkActor> { return pimpl->actor; }

auto PointsUnit::attach(Renderer& renderer) noexcept -> void {
    renderer.vtk_context().attach_unit(actor());
    renderer.vtk_context().attach_unit(coordinate_actor());
    renderer.vtk_context().attach_unit(coordinate_origin_actor());
}

auto PointsUnit::detach(Renderer& renderer) noexcept -> void {
    renderer.vtk_context().detach_unit(actor());
    renderer.vtk_context().detach_unit(coordinate_actor());
    renderer.vtk_context().detach_unit(coordinate_origin_actor());
}

auto PointsUnit::pick(Renderer& renderer, int display_x, int display_y) const noexcept
    -> std::optional<Renderer::Position> {
    auto picker = vtkSmartPointer<vtkPointPicker>::New();
    picker->PickFromListOn();
    picker->InitializePickList();
    picker->AddPickList(actor());

    auto* size    = renderer.vtk_context().window->GetSize();
    const auto ok = picker->Pick(static_cast<double>(display_x),
        static_cast<double>(size[1] - display_y - 1), 0.0, renderer.vtk_context().render);
    if (ok <= 0 || picker->GetPointId() < 0) {
        return std::nullopt;
    }

    auto point = std::array<double, 3> { };
    picker->GetPickPosition(point.data());
    return Renderer::Position { point[0], point[1], point[2] };
}

auto PointsUnit::coordinate_actor() noexcept -> SmartPointer<vtkProp> { return pimpl->coordinate; }

auto PointsUnit::coordinate_actor() const noexcept -> SmartPointer<vtkProp> {
    return pimpl->coordinate;
}

auto PointsUnit::coordinate_origin_actor() noexcept -> SmartPointer<vtkProp> {
    return pimpl->coordinate_origin_actor;
}

auto PointsUnit::coordinate_origin_actor() const noexcept -> SmartPointer<vtkProp> {
    return pimpl->coordinate_origin_actor;
}

auto PointsUnit::set_coordinate_visibility(bool on) noexcept -> void {
    if (pimpl->coordinate != nullptr) {
        pimpl->coordinate->SetVisibility(on);
    }
    if (pimpl->coordinate_origin_actor != nullptr) {
        pimpl->coordinate_origin_actor->SetVisibility(on);
    }
}

auto PointsUnit::set_coordinate_position(double x, double y, double z) noexcept -> void {
    if (pimpl->coordinate_transform != nullptr) {
        pimpl->coordinate_transform->Identity();
        pimpl->coordinate_transform->Translate(x, y, z);
    }
    if (pimpl->coordinate_origin_actor != nullptr) {
        pimpl->coordinate_origin_actor->SetPosition(x, y, z);
    }
}

auto PointsUnit::get_points_size() const noexcept -> std::size_t {
    return pimpl->points->GetNumberOfPoints();
}

PointsUnit::PointsUnit() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PointsUnit::~PointsUnit() noexcept = default;

}
