#include "core/handle/crop-box.impl.hh"

using namespace pcs;

CropBoxHandle::CropBoxHandle() noexcept
    : pimpl { std::make_unique<Impl>() } { }

CropBoxHandle::~CropBoxHandle() noexcept = default;

auto CropBoxHandle::set_bounds(
    double x_min, double x_max, double y_min, double y_max, double z_min, double z_max) noexcept
    -> void {
    pimpl->unit->set_bounds(x_min, x_max, y_min, y_max, z_min, z_max);
}

auto CropBoxHandle::set_visibility(bool on) noexcept -> void { pimpl->unit->set_visibility(on); }

auto CropBoxHandle::visibility() const noexcept -> bool { return pimpl->unit->visibility(); }

auto CropBoxHandle::attach_renderer(Renderer& renderer) noexcept -> void {
    renderer.attach(*pimpl->unit);
}

auto CropBoxHandle::detach_renderer(Renderer& renderer) noexcept -> void {
    renderer.detach(*pimpl->unit);
}
