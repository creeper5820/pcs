#pragma once

#include "core/events/common.hh"

#include <expected>
#include <tuple>
#include <vector>

#include <vtk/vtkPolyData.h>
#include <vtk/vtkSmartPointer.h>

namespace pcs::event {

struct ConvertModelToPointcloud {
    using Position = std::tuple<double, double, double>;
    using Result   = std::expected<std::vector<Position>, std::string>;

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
