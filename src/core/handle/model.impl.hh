#pragma once

#include "core/handle/model.hh"
#include "core/units/model.hh"

#include <vtk/vtkOBJReader.h>
#include <vtk/vtkPolyData.h>

using namespace pcs;

struct ModelHandle::Impl final {
    std::unique_ptr<ModelUnit> unit;
    vtkSmartPointer<vtkPolyData> data;

    auto load_from_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view> {
        auto reader = vtkSmartPointer<vtkOBJReader>::New();
        reader->SetFileName(path.c_str());
        reader->Update();

        data = vtkPolyData::New();
        data->ShallowCopy(reader->GetOutput());

        if (data == nullptr || data->GetNumberOfPoints() == 0) {
            return std::unexpected { "Failed to read model from filesystem" };
        }

        unit = std::make_unique<ModelUnit>(data);
        return { };
    }
};
