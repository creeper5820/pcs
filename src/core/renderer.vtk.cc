#include "renderer.vtk.hh"

namespace pcs {

auto Renderer::VtkContext::detach_unit(vtkProp* p) noexcept -> void {
    render->RemoveActor(p); //
}
auto Renderer::VtkContext::attach_unit(vtkProp* p) noexcept -> void {
    render->AddActor(p); //
}

}
