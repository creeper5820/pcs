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

auto Renderer::attach_points_unit(PointsUnit const& unit) noexcept -> void {
    pimpl->attach_points_unit(unit);
}

auto Renderer::detach_points_unit(PointsUnit const& unit) noexcept -> void {
    pimpl->detach_points_unit(unit);
}

auto Renderer::attach_model_unit(ModelUnit const& unit) noexcept -> void {
    pimpl->attach_model_unit(unit);
}

auto Renderer::detach_model_unit(ModelUnit const& unit) noexcept -> void {
    pimpl->detach_model_unit(unit);
}

auto Renderer::attach_png_map_unit(PngMapUnit const& unit) noexcept -> void {
    pimpl->attach_png_map_unit(unit);
}

auto Renderer::detach_png_map_unit(PngMapUnit const& unit) noexcept -> void {
    pimpl->detach_png_map_unit(unit);
}

auto Renderer::pick_points_unit(PointsUnit const& unit, int display_x, int display_y) noexcept
    -> std::optional<Position> {
    return pimpl->pick_points_unit(unit, display_x, display_y);
}

auto Renderer::pick_png_map_unit(PngMapUnit const& unit, int display_x, int display_y) noexcept
    -> std::optional<Position> {
    return pimpl->pick_png_map_unit(unit, display_x, display_y);
}

Renderer::Renderer() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Renderer::~Renderer() noexcept = default;

}
