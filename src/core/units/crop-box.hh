#pragma once

#include "common.hh"
#include "core/renderer.hh"

#include <vtk/vtkActor.h>

namespace pcs {

struct CropBoxUnit : public NormalUnit {
public:
    CropBoxUnit() noexcept;
    ~CropBoxUnit() noexcept;

    CropBoxUnit(const CropBoxUnit&)            = delete;
    CropBoxUnit& operator=(const CropBoxUnit&) = delete;

    auto actor() noexcept -> SmartPointer<vtkActor>;
    auto actor() const noexcept -> SmartPointer<vtkActor>;

    auto attach(Renderer&) noexcept -> void;
    auto detach(Renderer&) noexcept -> void;

    auto set_bounds(
        double x_min, double x_max, double y_min, double y_max, double z_min, double z_max) noexcept
        -> void;
    auto visibility() const noexcept -> bool;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}
