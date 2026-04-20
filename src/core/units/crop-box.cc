#include "crop-box.hh"

#include "core/renderer.vtk.hh"

#include <vtk/vtkCubeSource.h>
#include <vtk/vtkPolyDataMapper.h>

namespace pcs {

struct CropBoxUnit::Impl {
    SmartPointer<vtkCubeSource> cube;
    SmartPointer<vtkPolyDataMapper> mapper;
    SmartPointer<vtkActor> actor;

    auto initialize() -> void {
        cube = vtkCubeSource::New();
        cube->SetBounds(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);

        mapper = vtkPolyDataMapper::New();
        mapper->SetInputConnection(cube->GetOutputPort());

        actor = vtkActor::New();
        actor->SetMapper(mapper);
        actor->PickableOff();
        actor->GetProperty()->LightingOff();
        actor->GetProperty()->SetRepresentationToWireframe();
        actor->GetProperty()->SetLineWidth(2.0);
        actor->GetProperty()->SetColor(0.92, 0.24, 0.24);
        actor->SetVisibility(false);
    }
};

CropBoxUnit::CropBoxUnit() noexcept
    : pimpl { std::make_unique<Impl>() } {
    pimpl->initialize();
}

CropBoxUnit::~CropBoxUnit() noexcept = default;

auto CropBoxUnit::actor() noexcept -> SmartPointer<vtkActor> { return pimpl->actor; }

auto CropBoxUnit::actor() const noexcept -> SmartPointer<vtkActor> { return pimpl->actor; }

auto CropBoxUnit::attach(Renderer& renderer) noexcept -> void {
    renderer.vtk_context().attach_unit(actor());
}

auto CropBoxUnit::detach(Renderer& renderer) noexcept -> void {
    renderer.vtk_context().detach_unit(actor());
}

auto CropBoxUnit::set_bounds(
    double x_min, double x_max, double y_min, double y_max, double z_min, double z_max) noexcept
    -> void {
    pimpl->cube->SetBounds(x_min, x_max, y_min, y_max, z_min, z_max);
    pimpl->cube->Update();
    pimpl->actor->Modified();
}

auto CropBoxUnit::visibility() const noexcept -> bool {
    return pimpl->actor != nullptr && pimpl->actor->GetVisibility() != 0;
}

}
