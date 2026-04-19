#pragma once

#include "core/renderer.hh"
#include "core/renderer.vtk.hh"
#include "core/units/model.hh"
#include "core/units/png-map.hh"
#include "core/units/points.hh"

#include <spdlog/spdlog.h>

#include <array>
#include <optional>

#include <vtk/vtkCellPicker.h>
#include <vtk/vtkPointPicker.h>

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

    auto attach_points_unit(PointsUnit const& unit) noexcept {
        vtk_context.attach_unit(unit.actor());
    }

    auto detach_points_unit(PointsUnit const& unit) noexcept {
        vtk_context.detach_unit(unit.actor());
    }

    auto attach_model_unit(ModelUnit const& unit) noexcept {
        vtk_context.attach_unit(unit.actor());
    }

    auto detach_model_unit(ModelUnit const& unit) noexcept {
        vtk_context.detach_unit(unit.actor());
    }

    auto attach_png_map_unit(PngMapUnit const& unit) noexcept {
        vtk_context.attach_unit(unit.actor());
        vtk_context.attach_unit(unit.area_actor());
        vtk_context.attach_unit(unit.frame_actor());
        vtk_context.attach_unit(unit.frame_origin_actor());
    }

    auto detach_png_map_unit(PngMapUnit const& unit) noexcept {
        vtk_context.detach_unit(unit.actor());
        vtk_context.detach_unit(unit.area_actor());
        vtk_context.detach_unit(unit.frame_actor());
        vtk_context.detach_unit(unit.frame_origin_actor());
    }

    auto pick_points_unit(PointsUnit const& unit, int display_x, int display_y) noexcept
        -> std::optional<Renderer::Position> {
        auto picker = vtkSmartPointer<vtkPointPicker>::New();
        picker->PickFromListOn();
        picker->InitializePickList();
        picker->AddPickList(unit.actor());

        auto* size    = vtk_context.window->GetSize();
        const auto ok = picker->Pick(static_cast<double>(display_x),
            static_cast<double>(size[1] - display_y - 1), 0.0, vtk_context.render);
        if (ok <= 0 || picker->GetPointId() < 0) {
            return std::nullopt;
        }

        auto point = std::array<double, 3> { };
        picker->GetPickPosition(point.data());
        return Renderer::Position { point[0], point[1], point[2] };
    }

    auto pick_png_map_unit(PngMapUnit const& unit, int display_x, int display_y) noexcept
        -> std::optional<Renderer::Position> {
        auto picker = vtkSmartPointer<vtkCellPicker>::New();
        picker->SetTolerance(0.0005);
        picker->PickFromListOn();
        picker->InitializePickList();
        picker->AddPickList(unit.actor());

        auto* size    = vtk_context.window->GetSize();
        const auto ok = picker->Pick(static_cast<double>(display_x),
            static_cast<double>(size[1] - display_y - 1), 0.0, vtk_context.render);
        if (ok <= 0 || picker->GetCellId() < 0) {
            return std::nullopt;
        }

        auto point = std::array<double, 3> { };
        picker->GetPickPosition(point.data());
        return Renderer::Position { point[0], point[1], point[2] };
    }

    auto get_vtk_context() -> VtkContext& { return vtk_context; }

private:
    VtkContext vtk_context;
};

}
