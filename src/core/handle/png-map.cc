#include "core/handle/png-map.impl.hh"

using namespace pcs;

PngMapHandle::PngMapHandle() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PngMapHandle::~PngMapHandle() noexcept = default;

auto PngMapHandle::set_visibility(bool on) noexcept -> void { pimpl->unit->set_visibility(on); }

auto PngMapHandle::load_from_filesystem(std::string const& path) noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->load_from_filesystem(path);
}

auto PngMapHandle::load_from_data(PngMapData const& map) noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->load_from_data(map);
}

auto PngMapHandle::save_into_filesystem(std::string const& path) const noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->save_into_filesystem(path);
}

auto PngMapHandle::export_to_ros_directory(std::string const& directory) const noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->export_to_ros_directory(directory);
}

auto PngMapHandle::get_width() const noexcept -> std::size_t { return pimpl->data.width; }
auto PngMapHandle::get_height() const noexcept -> std::size_t { return pimpl->data.height; }
auto PngMapHandle::get_resolution() const noexcept -> double { return pimpl->data.resolution; }
auto PngMapHandle::get_plane_z() const noexcept -> double { return pimpl->data.plane_z; }
auto PngMapHandle::get_origin_x() const noexcept -> double { return pimpl->data.origin_x; }
auto PngMapHandle::get_origin_y() const noexcept -> double { return pimpl->data.origin_y; }
auto PngMapHandle::get_frame_config() const noexcept -> PngMapFrameConfig {
    return pimpl->data.frame_config;
}
auto PngMapHandle::set_frame_config(PngMapFrameConfig const& config) noexcept -> bool {
    return pimpl->set_frame_config(config);
}
auto PngMapHandle::transform_view() const noexcept -> PngMapTransformView {
    return make_png_map_transform_view(pimpl->data);
}

auto PngMapHandle::copy_pixels() const noexcept -> std::vector<std::uint8_t> {
    return pimpl->copy_pixels();
}

auto PngMapHandle::overwrite_pixels(std::vector<std::uint8_t> const& pixels) noexcept -> bool {
    return pimpl->overwrite_pixels(pixels);
}

auto PngMapHandle::preview_pixels(std::vector<std::uint8_t> const& pixels) noexcept -> bool {
    return pimpl->preview_pixels(pixels);
}

auto PngMapHandle::clear_preview() noexcept -> bool { return pimpl->clear_preview(); }

auto PngMapHandle::pick_plane_position(Renderer& renderer, int display_x,
    int display_y) const noexcept -> std::optional<PngMapHandle::Position> {
    return renderer.pick_png_map_unit(*pimpl->unit, display_x, display_y);
}

auto PngMapHandle::attach_renderer(Renderer& renderer) noexcept -> void {
    renderer.attach_png_map_unit(*pimpl->unit);
}

auto PngMapHandle::detach_renderer(Renderer& renderer) noexcept -> void {
    renderer.detach_png_map_unit(*pimpl->unit);
}
