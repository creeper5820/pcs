#pragma once

#include "core/renderer.hh"
#include "utility/pimpl.hh"

namespace pcs {

struct CropBoxHandle {
    PCS_PIMPL_DEFINITION(CropBoxHandle)

public:
    auto set_bounds(
        double x_min, double x_max, double y_min, double y_max, double z_min, double z_max) noexcept
        -> void;
    auto set_visibility(bool) noexcept -> void;
    auto visibility() const noexcept -> bool;

    auto attach_renderer(Renderer&) noexcept -> void;
    auto detach_renderer(Renderer&) noexcept -> void;
};

}
