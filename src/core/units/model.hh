#pragma once

#include "common.hh"
#include "core/map/model-data.hh"

#include <vtk/vtkActor.h>

#include <memory>

namespace pcs {

struct ModelUnit : public NormalUnit, TripleDUnit {
public:
    explicit ModelUnit(ModelData const&) noexcept;
    ~ModelUnit() noexcept;

    ModelUnit(const ModelUnit&)            = delete;
    ModelUnit& operator=(const ModelUnit&) = delete;

    auto actor() noexcept -> SmartPointer<vtkActor>;
    auto actor() const noexcept -> SmartPointer<vtkActor>;

    auto get_points_size() const noexcept -> std::size_t;
    auto get_polys_size() const noexcept -> std::size_t;

private:
    auto initialize(ModelData const&) noexcept -> void;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}
