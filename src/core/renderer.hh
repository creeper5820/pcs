#pragma once
#include "utility/pimpl.hh"
#include "utility/qt_binding.hh"

namespace pcs {

class Renderer final {
    PCS_PIMPL_DEFINITION(Renderer)

public:
    struct VtkContext;
    auto vtk_context() noexcept -> VtkContext&;

    auto render_window() noexcept -> void;

    auto reset_camera() noexcept -> void;

    auto connect_ui(QtVtkWindow&) noexcept -> void;

    auto set_background(double r, double g, double b) noexcept -> void;
};

}
