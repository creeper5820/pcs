#include "core/handle/model.impl.hh"

using namespace pcs;

ModelHandle::ModelHandle() noexcept
    : pimpl { std::make_unique<Impl>() } { }

ModelHandle::~ModelHandle() noexcept = default;

auto ModelHandle::set_position(double x, double y, double z) noexcept -> void {
    pimpl->unit->set_position(x, y, z);
}

auto ModelHandle::get_position() const noexcept -> Position {
    auto position = pimpl->unit->get_position();
    return std::make_tuple(position[0], position[1], position[2]);
}

auto ModelHandle::set_overall_color(double r, double g, double b) noexcept -> void {
    pimpl->unit->set_color(r, g, b);
}

auto ModelHandle::get_overall_color() const noexcept -> std::tuple<double, double, double> {
    auto c = pimpl->unit->actor()->GetProperty()->GetColor();
    return std::make_tuple(c[0], c[1], c[2]);
}

auto ModelHandle::get_points_size() const noexcept -> std::size_t {
    return pimpl->unit->get_points_size();
}

auto ModelHandle::get_polys_size() const noexcept -> std::size_t {
    return pimpl->unit->get_polys_size();
}

auto ModelHandle::set_visibility(bool on) noexcept -> void { pimpl->unit->set_visibility(on); }

auto ModelHandle::load_from_filesystem(std::string const& path) noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->load_from_filesystem(path);
}

auto ModelHandle::save_into_filesystem(std::string const&) noexcept
    -> std::expected<void, std::string_view> {
    return std::unexpected { "Save not supported for model asset" };
}

auto ModelHandle::model_data() const noexcept -> ModelData const& { return pimpl->data; }

auto ModelHandle::clone(std::string const&) const noexcept
    -> std::expected<std::unique_ptr<ModelHandle>, std::string> {
    auto cloned      = std::make_unique<ModelHandle>();
    auto load_result = cloned->pimpl->load_from_data(model_data());
    if (!load_result.has_value()) {
        return std::unexpected { std::string(load_result.error()) };
    }

    const auto [r, g, b] = get_overall_color();
    cloned->set_overall_color(r, g, b);
    return cloned;
}

auto ModelHandle::attach_renderer(Renderer& r) noexcept -> void {
    r.attach(*pimpl->unit);
}

auto ModelHandle::detach_renderer(Renderer& r) noexcept -> void {
    r.detach(*pimpl->unit);
}
