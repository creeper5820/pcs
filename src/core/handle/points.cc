#include "core/handle/points.impl.hh"
#include "core/renderer.vtk.hh"

using namespace pcs;

PointsHandle::PointsHandle() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PointsHandle::~PointsHandle() noexcept = default;

auto PointsHandle::set_position(double x, double y, double z) noexcept -> void {
    pimpl->unit->set_position(x, y, z);
}
auto PointsHandle::get_position() const noexcept -> Position {
    auto position = pimpl->unit->get_position();
    return std::make_tuple(position[0], position[1], position[2]);
}

auto PointsHandle::set_overall_color(double r, double g, double b, double a) noexcept -> void {
    pimpl->unit->set_color(r, g, b);
    pimpl->unit->set_alpha(a);
}
auto PointsHandle::get_overall_color() const noexcept
    -> std::tuple<double, double, double, double> {
    auto c = pimpl->unit->actor()->GetProperty()->GetColor();
    auto a = pimpl->unit->actor()->GetProperty()->GetOpacity();
    return std::make_tuple(c[0], c[1], c[2], a);
}

auto PointsHandle::get_points_size() const noexcept -> std::size_t {
    return pimpl->unit->get_points_size();
}

auto PointsHandle::get_positions() const noexcept -> std::vector<Position> {
    return pimpl->get_positions();
}

auto PointsHandle::set_visibility(bool on) noexcept -> void {
    pimpl->unit->set_visibility(on); //
}

auto PointsHandle::attach_renderer(Renderer& r) noexcept -> void {
    r.vtk_context().attach_unit(pimpl->unit->actor());
}
auto PointsHandle::detach_renderer(Renderer& r) noexcept -> void {
    r.vtk_context().detach_unit(pimpl->unit->actor());
}

auto PointsHandle::load_from_filesystem(std::string const& path) noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->load_from_filesystem(path);
}
auto PointsHandle::load_from_positions(std::vector<Position> const& points) noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->load_from_positions(points);
}
auto PointsHandle::save_into_filesystem(std::string const& path) noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->save_into_filesystem(path);
}
