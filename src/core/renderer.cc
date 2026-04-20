#include "renderer.impl.hh"

namespace pcs {

auto Renderer::vtk_context() noexcept -> VtkContext& {
    return pimpl->get_vtk_context(); //
}
auto Renderer::connect_ui(QVTKOpenGLNativeWidget& ui) noexcept -> void {
    pimpl->connect_ui(ui); //
}
auto Renderer::render_window() noexcept -> void {
    pimpl->get_vtk_context().render_window(); //
}
auto Renderer::reset_camera() noexcept -> void {
    pimpl->get_vtk_context().render->ResetCamera();
    pimpl->get_vtk_context().render_window();
}
auto Renderer::set_background(double r, double g, double b) noexcept -> void {
    pimpl->set_background(r, g, b);
}

Renderer::Renderer() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Renderer::~Renderer() noexcept = default;

}
