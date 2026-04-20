#pragma once
#include "utility/pimpl.hh"
#include "utility/qt_binding.hh"

#include <concepts>
#include <optional>
#include <tuple>

namespace pcs {

class Renderer;

template <class Unit>
concept render_unit_trait = requires(Unit& unit, Renderer& renderer) {
    { unit.attach(renderer) } -> std::same_as<void>;
    { unit.detach(renderer) } -> std::same_as<void>;
};

class PointsUnit;
class ModelUnit;
class PngMapUnit;

class Renderer final {
    PCS_PIMPL_DEFINITION(Renderer)

public:
    struct VtkContext;

    using Position = std::tuple<double, double, double>;

    auto attach(render_unit_trait auto& unit) noexcept { unit.attach(*this); }
    auto detach(render_unit_trait auto& unit) noexcept { unit.detach(*this); }

    auto vtk_context() noexcept -> VtkContext&;

    auto render_window() noexcept -> void;

    auto reset_camera() noexcept -> void;

    auto connect_ui(QtVtkWindow&) noexcept -> void;

    auto set_background(double r, double g, double b) noexcept -> void;

    template <class Unit>
    requires requires(Unit const& unit, Renderer& renderer, int x, int y) {
        { unit.pick(renderer, x, y) } -> std::same_as<std::optional<Position>>;
    }
    auto pick(Unit const& unit, int display_x, int display_y) noexcept -> std::optional<Position> {
        return unit.pick(*this, display_x, display_y);
    }
};

}
