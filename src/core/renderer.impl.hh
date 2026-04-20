#pragma once

#include "core/renderer.hh"
#include "core/renderer.vtk.hh"

#include <spdlog/spdlog.h>

namespace pcs {

struct Renderer::Impl {
public:
    explicit Impl() noexcept
        : vtk_context { } {
        spdlog::info("Vtk context has been created now");
    }

    auto connect_ui(QtVtkWindow& ui) {
        ui.setRenderWindow(vtk_context.window);
        spdlog::info("Renderer(vtk backend) is connected with qt window");
    }

    auto set_background(auto r, auto g, auto b) noexcept {
        vtk_context.render->SetBackground(r, g, b);
    }

    auto get_vtk_context() -> VtkContext& { return vtk_context; }

private:
    VtkContext vtk_context;
};

}
