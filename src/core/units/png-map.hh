#pragma once

#include "common.hh"
#include "core/map/png-map-data.hh"
#include "core/renderer.hh"

#include <vtk/vtkActor.h>
#include <vtk/vtkProp.h>

namespace pcs {

struct PngMapUnit : public NormalUnit {
public:
    explicit PngMapUnit(PngMapData const&) noexcept;
    ~PngMapUnit() noexcept;

    PngMapUnit(const PngMapUnit&)            = delete;
    PngMapUnit& operator=(const PngMapUnit&) = delete;

    auto actor() noexcept -> SmartPointer<vtkActor>;
    auto area_actor() noexcept -> SmartPointer<vtkActor>;
    auto frame_actor() noexcept -> vtkProp*;
    auto frame_origin_actor() noexcept -> SmartPointer<vtkActor>;
    auto actor() const noexcept -> SmartPointer<vtkActor>;
    auto area_actor() const noexcept -> SmartPointer<vtkActor>;
    auto frame_actor() const noexcept -> vtkProp*;
    auto frame_origin_actor() const noexcept -> SmartPointer<vtkActor>;

    auto attach(Renderer&) noexcept -> void;
    auto detach(Renderer&) noexcept -> void;
    auto pick(Renderer&, int display_x, int display_y) const noexcept
        -> std::optional<Renderer::Position>;

    auto set_visibility(bool on) noexcept -> void;
    auto set_frame_visibility(bool on) noexcept -> void;
    auto frame_visibility() const noexcept -> bool;
    auto set_area_visibility(bool on) noexcept -> void;
    auto area_visibility() const noexcept -> bool;
    auto update_pixels(std::vector<std::uint8_t> const& pixels) noexcept -> bool;
    auto update_frame_config(PngMapData const& map) noexcept -> void;

private:
    auto initialize(PngMapData const&) noexcept -> void;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}
