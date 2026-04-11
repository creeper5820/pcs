#include "points.hh"

#include <vtk/vtkPolyData.h>
#include <vtk/vtkPolyDataMapper.h>
#include <vtk/vtkRenderer.h>
#include <vtk/vtkVertexGlyphFilter.h>

namespace pcs {

struct PointsUnit::Impl {
    SmartPointer<vtkPoints> points;
    SmartPointer<vtkPolyData> data;
    SmartPointer<vtkPolyDataMapper> mapper;
    SmartPointer<vtkActor> actor;

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
    }
};

auto PointsUnit::initialize(SmartPointer<vtkPoints> points) noexcept -> void {
    pimpl->initialize(points);
}

auto PointsUnit::actor() noexcept -> SmartPointer<vtkActor> { return pimpl->actor; }

auto PointsUnit::get_points_size() const noexcept -> std::size_t {
    return pimpl->points->GetNumberOfPoints();
}

PointsUnit::PointsUnit() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PointsUnit::~PointsUnit() noexcept = default;

}
