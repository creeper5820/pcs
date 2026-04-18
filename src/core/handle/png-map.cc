#include "core/handle/png-map.impl.hh"
#include "core/renderer.vtk.hh"

using namespace pcs;

PngMapHandle::PngMapHandle() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PngMapHandle::~PngMapHandle() noexcept = default;

auto PngMapHandle::set_visibility(bool on) noexcept -> void { pimpl->unit->set_visibility(on); }

auto PngMapHandle::load_from_data(PngMapData const& map) noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->load_from_data(map);
}

auto PngMapHandle::save_into_filesystem(std::string const& path) const noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->save_into_filesystem(path);
}

auto PngMapHandle::get_width() const noexcept -> std::size_t { return pimpl->data.width; }
auto PngMapHandle::get_height() const noexcept -> std::size_t { return pimpl->data.height; }
auto PngMapHandle::get_resolution() const noexcept -> double { return pimpl->data.resolution; }
auto PngMapHandle::get_plane_z() const noexcept -> double { return pimpl->data.plane_z; }

auto PngMapHandle::attach_renderer(Renderer& renderer) noexcept -> void {
    renderer.vtk_context().attach_unit(pimpl->unit->actor());
    renderer.vtk_context().attach_unit(pimpl->unit->area_actor());
}

auto PngMapHandle::detach_renderer(Renderer& renderer) noexcept -> void {
    renderer.vtk_context().detach_unit(pimpl->unit->actor());
    renderer.vtk_context().detach_unit(pimpl->unit->area_actor());
}
