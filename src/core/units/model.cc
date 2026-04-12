#include "model.hh"

#include <vtk/vtkPolyDataMapper.h>

namespace pcs {

struct ModelUnit::Impl {
    SmartPointer<vtkPolyData> data;
    SmartPointer<vtkPolyDataMapper> mapper;
    SmartPointer<vtkActor> actor;

    auto initialize(SmartPointer<vtkPolyData> poly_data) {
        data = std::move(poly_data);

        mapper = vtkPolyDataMapper::New();
        mapper->SetInputData(data);

        actor = vtkActor::New();
        actor->SetMapper(mapper);
    }
};

ModelUnit::ModelUnit(SmartPointer<vtkPolyData> poly_data) noexcept
    : pimpl { std::make_unique<Impl>() } {
    initialize(std::move(poly_data));
}

auto ModelUnit::initialize(SmartPointer<vtkPolyData> poly_data) noexcept -> void {
    pimpl->initialize(std::move(poly_data));
}

auto ModelUnit::actor() noexcept -> SmartPointer<vtkActor> { return pimpl->actor; }

auto ModelUnit::get_points_size() const noexcept -> std::size_t {
    return pimpl->data->GetNumberOfPoints();
}

auto ModelUnit::get_polys_size() const noexcept -> std::size_t {
    return pimpl->data->GetNumberOfPolys();
}

ModelUnit::~ModelUnit() noexcept = default;

}
