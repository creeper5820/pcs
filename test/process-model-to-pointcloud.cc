#include "core/events/process/model-to-pointcloud.hh"

#include <vtk/vtkCellArray.h>
#include <vtk/vtkPoints.h>
#include <vtk/vtkPolyData.h>
#include <vtk/vtkSmartPointer.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

auto main() -> int {
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(0.0, 0.0, 0.0);
    points->InsertNextPoint(1000.0, 0.0, 0.0);
    points->InsertNextPoint(1000.0, 1000.0, 0.0);
    points->InsertNextPoint(0.0, 1000.0, 0.0);

    auto triangles                 = vtkSmartPointer<vtkCellArray>::New();
    constexpr auto first_triangle  = std::array<vtkIdType, 3> { 0, 1, 2 };
    constexpr auto second_triangle = std::array<vtkIdType, 3> { 0, 2, 3 };
    triangles->InsertNextCell(first_triangle.size(), first_triangle.data());
    triangles->InsertNextCell(second_triangle.size(), second_triangle.data());

    auto poly_data = vtkSmartPointer<vtkPolyData>::New();
    poly_data->SetPoints(points);
    poly_data->SetPolys(triangles);

    auto context             = std::make_unique<pcs::event::ConvertModelToPointcloud::Context>();
    context->poly_data       = poly_data;
    context->sample_distance = 0.25;
    context->unit_scale      = 0.001;

    auto result = pcs::event::ConvertModelToPointcloud::runtime_exec(std::move(context));
    if (!result.has_value()) {
        std::cerr << "conversion failed: " << result.error() << '\n';
        return EXIT_FAILURE;
    }

    const auto points_size = result.value().size();
    if (points_size <= 4) {
        std::cerr << "sampled points too few: " << points_size << '\n';
        return EXIT_FAILURE;
    }

    auto max_abs_coordinate = 0.0;
    for (const auto& [x, y, z] : result.value()) {
        max_abs_coordinate = std::max(max_abs_coordinate, std::abs(x));
        max_abs_coordinate = std::max(max_abs_coordinate, std::abs(y));
        max_abs_coordinate = std::max(max_abs_coordinate, std::abs(z));
    }

    if (max_abs_coordinate > 2.0) {
        std::cerr << "unit scaling invalid, max coordinate: " << max_abs_coordinate << '\n';
        return EXIT_FAILURE;
    }

    std::cout << "sampled points: " << points_size << '\n';
    return EXIT_SUCCESS;
}
