#pragma once

#include "common.hh"
#include "core/map/png-map-data.hh"

#include <vtk/vtkActor.h>

namespace pcs {

struct PngMapUnit : public NormalUnit {
public:
    explicit PngMapUnit(PngMapData const&) noexcept;
    ~PngMapUnit() noexcept;

    PngMapUnit(const PngMapUnit&)            = delete;
    PngMapUnit& operator=(const PngMapUnit&) = delete;

    auto actor() noexcept -> SmartPointer<vtkActor>;
    auto area_actor() noexcept -> SmartPointer<vtkActor>;

    auto set_visibility(bool on) noexcept -> void;

private:
    auto initialize(PngMapData const&) noexcept -> void;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}
