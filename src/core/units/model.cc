#include "model.hh"

#include "core/renderer.vtk.hh"

#include <vtk/vtkCellArray.h>
#include <vtk/vtkPoints.h>
#include <vtk/vtkPolyData.h>
#include <vtk/vtkPolyDataMapper.h>

#include <array>

namespace pcs {

struct ModelUnit::Impl {
    SmartPointer<vtkPolyData> data;
    SmartPointer<vtkPolyDataMapper> mapper;
    SmartPointer<vtkActor> actor;

    auto initialize(ModelData const& model) {
        auto points = vtkSmartPointer<vtkPoints>::New();
        points->SetDataTypeToDouble();

        for (auto const& vertex : model.vertices) {
            points->InsertNextPoint(vertex[0], vertex[1], vertex[2]);
        }

        auto polys = vtkSmartPointer<vtkCellArray>::New();
        for (auto const& face : model.faces) {
            const auto triangle = std::array<vtkIdType, 3> {
                static_cast<vtkIdType>(face[0]),
                static_cast<vtkIdType>(face[1]),
                static_cast<vtkIdType>(face[2]),
            };

            polys->InsertNextCell(static_cast<vtkIdType>(triangle.size()), triangle.data());
        }

        data = vtkPolyData::New();
        data->SetPoints(points);
        data->SetPolys(polys);

        mapper = vtkPolyDataMapper::New();
        mapper->SetInputData(data);

        actor = vtkActor::New();
        actor->SetMapper(mapper);
    }
};

ModelUnit::ModelUnit(ModelData const& model) noexcept
    : pimpl { std::make_unique<Impl>() } {
    initialize(model);
}

auto ModelUnit::initialize(ModelData const& model) noexcept -> void { pimpl->initialize(model); }

auto ModelUnit::actor() noexcept -> SmartPointer<vtkActor> { return pimpl->actor; }

auto ModelUnit::actor() const noexcept -> SmartPointer<vtkActor> { return pimpl->actor; }

auto ModelUnit::attach(Renderer& renderer) noexcept -> void {
    renderer.vtk_context().attach_unit(actor());
}

auto ModelUnit::detach(Renderer& renderer) noexcept -> void {
    renderer.vtk_context().detach_unit(actor());
}

auto ModelUnit::get_points_size() const noexcept -> std::size_t {
    return pimpl->data->GetNumberOfPoints();
}

auto ModelUnit::get_polys_size() const noexcept -> std::size_t {
    return pimpl->data->GetNumberOfPolys();
}

ModelUnit::~ModelUnit() noexcept = default;

}
