#pragma once
#include "core/renderer.hh"
#include <vtk/vtkGenericOpenGLRenderWindow.h>
#include <vtk/vtkRenderer.h>

namespace pcs {

struct Renderer::VtkContext {
    template <typename T>
    using SmartPointer = vtkSmartPointer<T>;

    using Render = vtkRenderer;
    using Window = vtkGenericOpenGLRenderWindow;

    SmartPointer<Render> render;
    SmartPointer<Window> window;

    static inline auto unique_id = std::size_t { 0 };

    explicit VtkContext() {

        const auto name = std::format("window-{}", unique_id++);

        render = Render::New();
        window = Window::New();

        window->AddRenderer(render);
        window->SetWindowName(name.c_str());
    }

    auto detach_unit(vtkProp* p) noexcept -> void;
    auto attach_unit(vtkProp* p) noexcept -> void;

    auto render_window() { window->Render(); }
};

}
