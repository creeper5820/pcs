#include "model-to-pointcloud.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <vtk/vtkPoints.h>
#include <vtk/vtkPolyDataPointSampler.h>
#include <vtk/vtkTriangleFilter.h>

namespace {

auto calculate_default_sample_distance(vtkPolyData* poly_data) noexcept -> double {
    auto bounds = std::array<double, 6> { };
    poly_data->GetBounds(bounds.data());

    const auto dx       = bounds[1] - bounds[0];
    const auto dy       = bounds[3] - bounds[2];
    const auto dz       = bounds[5] - bounds[4];
    const auto diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (diagonal <= 0.0) {
        return 0.01;
    }

    return std::max(diagonal / 200.0, 1e-3);
}

}

namespace pcs::event {

auto ConvertModelToPointcloud::runtime_exec(std::unique_ptr<Context> context) noexcept -> Result {
    if (context == nullptr || context->poly_data == nullptr) {
        return std::unexpected { "Model polydata is not loaded" };
    }

    auto triangle = vtkSmartPointer<vtkTriangleFilter>::New();
    triangle->PassVertsOff();
    triangle->PassLinesOff();
    triangle->SetInputData(context->poly_data);
    triangle->Update();

    auto sampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
    sampler->SetInputConnection(triangle->GetOutputPort());
    sampler->SetPointGenerationModeToRegular();
    sampler->GenerateVertexPointsOff();
    sampler->GenerateEdgePointsOn();
    sampler->GenerateInteriorPointsOn();
    sampler->GenerateVerticesOff();
    sampler->SetDistance(context->sample_distance > 0.0
            ? context->sample_distance
            : calculate_default_sample_distance(context->poly_data));
    sampler->Update();

    auto sampled = sampler->GetOutput();
    auto* points = sampled->GetPoints();

    if (points == nullptr || points->GetNumberOfPoints() == 0) {
        return std::unexpected { "Failed to sample pointcloud from model" };
    }

    auto positions = std::vector<std::tuple<double, double, double>> { };
    positions.reserve(points->GetNumberOfPoints());

    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
        auto point = std::array<double, 3> { };
        points->GetPoint(i, point.data());
        positions.emplace_back(point[0], point[1], point[2]);
    }

    return positions;
}

}
