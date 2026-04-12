#pragma once

#include "common.hh"

#include <vtk/vtkActor.h>
#include <vtk/vtkPolyData.h>

#include <memory>

namespace pcs {

struct ModelUnit : public NormalUnit, TripleDUnit {
public:
    explicit ModelUnit(SmartPointer<vtkPolyData>) noexcept;
    ~ModelUnit() noexcept;

    ModelUnit(const ModelUnit&)            = delete;
    ModelUnit& operator=(const ModelUnit&) = delete;

    auto actor() noexcept -> SmartPointer<vtkActor>;

    auto get_points_size() const noexcept -> std::size_t;
    auto get_polys_size() const noexcept -> std::size_t;

private:
    auto initialize(SmartPointer<vtkPolyData>) noexcept -> void;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}
