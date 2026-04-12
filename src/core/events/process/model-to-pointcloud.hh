#pragma once

#include "core/events/common.hh"
#include "core/handle/points.hh"

#include <expected>

#include <vtk/vtkPolyData.h>
#include <vtk/vtkSmartPointer.h>

namespace pcs::event {

struct ConvertModelToPointcloud {
    using Result = std::expected<std::unique_ptr<PointsHandle>, std::string>;

    struct Context {
        static constexpr EventMeta meta {
            .name      = "Convert Model To Pointcloud",
            .consuming = true,
        };

        vtkSmartPointer<vtkPolyData> poly_data;
        double sample_distance = 0.0;
    };

    static auto runtime_exec(std::unique_ptr<Context>) noexcept -> Result;
};

}
