#include "model-to-pointcloud.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
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

auto apply_max_points_limit(std::vector<pcs::event::ConvertModelToPointcloud::Position>& positions,
    std::size_t max_points) noexcept -> void {
    if (max_points == 0 || positions.size() <= max_points) {
        return;
    }

    auto reduced = std::vector<pcs::event::ConvertModelToPointcloud::Position> { };
    reduced.reserve(max_points);

    if (max_points == 1) {
        reduced.push_back(positions.front());
        positions = std::move(reduced);
        return;
    }

    const auto step =
        static_cast<double>(positions.size() - 1) / static_cast<double>(max_points - 1);
    for (std::size_t i = 0; i < max_points; ++i) {
        const auto index = static_cast<std::size_t>(std::round(step * static_cast<double>(i)));
        reduced.push_back(positions[std::min(index, positions.size() - 1)]);
    }

    positions = std::move(reduced);
}

}

namespace pcs::event {

auto ConvertModelToPointcloud::runtime_exec(std::unique_ptr<Context> context) noexcept -> Result {
    if (context == nullptr || context->poly_data == nullptr) {
        return std::unexpected { "Model polydata is not loaded" };
    }

    const auto& parameters = context->parameters;
    if (!parameters.include_edge_points && !parameters.include_interior_points
        && !parameters.include_vertex_points) {
        return std::unexpected { "At least one sampling source must be enabled" };
    }

    if (parameters.density <= 0.0) {
        return std::unexpected { "Density must be greater than 0" };
    }

    if (parameters.unit_scale <= 0.0) {
        return std::unexpected { "Unit scale must be greater than 0" };
    }

    const auto base_distance = parameters.sample_distance > 0.0
        ? parameters.sample_distance
        : calculate_default_sample_distance(context->poly_data);

    const auto sample_distance = std::max(base_distance / std::sqrt(parameters.density), 1e-5);

    auto triangle = vtkSmartPointer<vtkTriangleFilter>::New();
    triangle->PassVertsOff();
    triangle->PassLinesOff();
    triangle->SetInputData(context->poly_data);
    triangle->Update();

    auto sampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
    sampler->SetInputConnection(triangle->GetOutputPort());
    sampler->SetPointGenerationModeToRegular();
    if (parameters.include_vertex_points) {
        sampler->GenerateVertexPointsOn();
    } else {
        sampler->GenerateVertexPointsOff();
    }

    if (parameters.include_edge_points) {
        sampler->GenerateEdgePointsOn();
    } else {
        sampler->GenerateEdgePointsOff();
    }

    if (parameters.include_interior_points) {
        sampler->GenerateInteriorPointsOn();
    } else {
        sampler->GenerateInteriorPointsOff();
    }

    sampler->GenerateVerticesOff();
    sampler->SetDistance(sample_distance);
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
        positions.emplace_back(point[0] * parameters.unit_scale, point[1] * parameters.unit_scale,
            point[2] * parameters.unit_scale);
    }

    apply_max_points_limit(positions, parameters.max_points);

    return positions;
}

}
