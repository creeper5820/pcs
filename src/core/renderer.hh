#pragma once
#include "utility/pimpl.hh"
#include "utility/qt_binding.hh"

#include <optional>
#include <tuple>

namespace pcs {

class PointsUnit;
class ModelUnit;
class PngMapUnit;

class Renderer final {
    PCS_PIMPL_DEFINITION(Renderer)

public:
    struct VtkContext;

    using Position = std::tuple<double, double, double>;

    auto vtk_context() noexcept -> VtkContext&;

    auto render_window() noexcept -> void;

    auto reset_camera() noexcept -> void;

    auto connect_ui(QtVtkWindow&) noexcept -> void;

    auto set_background(double r, double g, double b) noexcept -> void;

    auto attach_points_unit(PointsUnit const&) noexcept -> void;
    auto detach_points_unit(PointsUnit const&) noexcept -> void;

    auto attach_model_unit(ModelUnit const&) noexcept -> void;
    auto detach_model_unit(ModelUnit const&) noexcept -> void;

    auto attach_png_map_unit(PngMapUnit const&) noexcept -> void;
    auto detach_png_map_unit(PngMapUnit const&) noexcept -> void;

    auto pick_points_unit(PointsUnit const&, int display_x, int display_y) noexcept
        -> std::optional<Position>;

    auto pick_png_map_unit(PngMapUnit const&, int display_x, int display_y) noexcept
        -> std::optional<Position>;
};

}
