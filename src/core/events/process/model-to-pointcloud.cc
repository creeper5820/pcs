#include "model-to-pointcloud.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>

#include <vtk/vtkCellArray.h>
#include <vtk/vtkCleanPolyData.h>
#include <vtk/vtkPoints.h>
#include <vtk/vtkPolyData.h>
#include <vtk/vtkPolyDataPointSampler.h>
#include <vtk/vtkRemoveDuplicatePolys.h>
#include <vtk/vtkSelectEnclosedPoints.h>
#include <vtk/vtkTriangleFilter.h>

namespace {

using Position3 = std::array<double, 3>;

struct GridKey {
    std::int64_t x { 0 };
    std::int64_t y { 0 };
    std::int64_t z { 0 };

    auto operator==(GridKey const& other) const noexcept -> bool {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridKeyHash {
    auto operator()(GridKey const& key) const noexcept -> std::size_t {
        auto h = std::size_t { };
        h ^= static_cast<std::size_t>(key.x) * 73856093U;
        h ^= static_cast<std::size_t>(key.y) * 19349663U;
        h ^= static_cast<std::size_t>(key.z) * 83492791U;
        return h;
    }
};

auto build_poly_data(pcs::ModelData const& model) -> vtkSmartPointer<vtkPolyData> {
    if (model.vertices.empty()) {
        return nullptr;
    }

    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetDataTypeToDouble();

    for (auto const& vertex : model.vertices) {
        points->InsertNextPoint(vertex[0], vertex[1], vertex[2]);
    }

    auto polys = vtkSmartPointer<vtkCellArray>::New();
    for (auto const& face : model.faces) {
        if (face[0] >= model.vertices.size() || face[1] >= model.vertices.size()
            || face[2] >= model.vertices.size()) {
            continue;
        }

        const auto triangle = std::array<vtkIdType, 3> {
            static_cast<vtkIdType>(face[0]),
            static_cast<vtkIdType>(face[1]),
            static_cast<vtkIdType>(face[2]),
        };
        polys->InsertNextCell(static_cast<vtkIdType>(triangle.size()), triangle.data());
    }

    auto poly_data = vtkSmartPointer<vtkPolyData>::New();
    poly_data->SetPoints(points);
    poly_data->SetPolys(polys);
    return poly_data;
}

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

auto calculate_bounds_diagonal(vtkPolyData* poly_data) noexcept -> double {
    if (poly_data == nullptr) {
        return 0.0;
    }

    auto bounds = std::array<double, 6> { };
    poly_data->GetBounds(bounds.data());

    const auto dx = bounds[1] - bounds[0];
    const auto dy = bounds[3] - bounds[2];
    const auto dz = bounds[5] - bounds[4];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

auto make_grid_key(Position3 const& point, double cell_size) noexcept -> GridKey {
    const auto safe_cell = std::max(cell_size, 1e-6);

    return {
        static_cast<std::int64_t>(std::floor(point[0] / safe_cell)),
        static_cast<std::int64_t>(std::floor(point[1] / safe_cell)),
        static_cast<std::int64_t>(std::floor(point[2] / safe_cell)),
    };
}

auto deduplicate_overlapped_samples(std::vector<Position3>& samples, double cell_size) noexcept
    -> void {
    if (samples.empty()) {
        return;
    }

    auto reduced = std::vector<Position3> { };
    reduced.reserve(samples.size());

    auto occupied = std::unordered_map<GridKey, std::size_t, GridKeyHash> { };
    occupied.reserve(samples.size());

    for (auto const& point : samples) {
        auto key = make_grid_key(point, cell_size);
        if (!occupied.contains(key)) {
            occupied.emplace(key, reduced.size());
            reduced.push_back(point);
        }
    }

    samples = std::move(reduced);
}

auto remove_internal_surface_samples(
    std::vector<Position3>& samples, vtkPolyData* surface, double probe_offset) noexcept -> void {
    if (samples.empty() || surface == nullptr) {
        return;
    }

    if (vtkSelectEnclosedPoints::IsSurfaceClosed(surface) <= 0) {
        return;
    }

    auto enclosed = vtkSmartPointer<vtkSelectEnclosedPoints>::New();
    enclosed->SetTolerance(1e-6);
    enclosed->Initialize(surface);

    static constexpr auto directions = std::array<Position3, 6> {
        Position3 { 1.0, 0.0, 0.0 },
        Position3 { -1.0, 0.0, 0.0 },
        Position3 { 0.0, 1.0, 0.0 },
        Position3 { 0.0, -1.0, 0.0 },
        Position3 { 0.0, 0.0, 1.0 },
        Position3 { 0.0, 0.0, -1.0 },
    };

    auto filtered = std::vector<Position3> { };
    filtered.reserve(samples.size());

    for (auto const& point : samples) {
        auto has_outside_direction = false;

        for (auto const& direction : directions) {
            auto probe = Position3 {
                point[0] + direction[0] * probe_offset,
                point[1] + direction[1] * probe_offset,
                point[2] + direction[2] * probe_offset,
            };

            if (enclosed->IsInsideSurface(probe[0], probe[1], probe[2]) == 0) {
                has_outside_direction = true;
                break;
            }
        }

        if (has_outside_direction) {
            filtered.push_back(point);
        }
    }

    enclosed->Complete();

    if (!filtered.empty()) {
        samples = std::move(filtered);
    }
}

auto regularize_density(std::vector<Position3>& samples, double cell_size) noexcept -> void {
    if (samples.empty()) {
        return;
    }

    struct Accumulator {
        Position3 sum { 0.0, 0.0, 0.0 };
        std::size_t count { 0 };
    };

    auto buckets = std::unordered_map<GridKey, Accumulator, GridKeyHash> { };
    buckets.reserve(samples.size());

    for (auto const& point : samples) {
        auto key  = make_grid_key(point, cell_size);
        auto& acc = buckets[key];
        acc.sum[0] += point[0];
        acc.sum[1] += point[1];
        acc.sum[2] += point[2];
        acc.count += 1;
    }

    auto reduced = std::vector<Position3> { };
    reduced.reserve(buckets.size());

    for (auto const& [_, acc] : buckets) {
        if (acc.count == 0) {
            continue;
        }

        const auto inv_count = 1.0 / static_cast<double>(acc.count);
        reduced.push_back(
            { acc.sum[0] * inv_count, acc.sum[1] * inv_count, acc.sum[2] * inv_count });
    }

    samples = std::move(reduced);
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
    if (context == nullptr || context->model.vertices.empty()) {
        return std::unexpected { "Model data is not loaded" };
    }

    auto poly_data = build_poly_data(context->model);
    if (poly_data == nullptr || poly_data->GetNumberOfPoints() == 0) {
        return std::unexpected { "Model polydata is not loaded" };
    }

    const auto& parameters = context->parameters;
    if (parameters.density <= 0.0) {
        return std::unexpected { "Density must be greater than 0" };
    }

    if (parameters.unit_scale <= 0.0) {
        return std::unexpected { "Unit scale must be greater than 0" };
    }

    const auto base_distance      = parameters.sample_distance > 0.0
        ? parameters.sample_distance
        : calculate_default_sample_distance(poly_data);
    const auto effective_distance = std::max(base_distance / std::sqrt(parameters.density), 1e-5);

    auto triangle = vtkSmartPointer<vtkTriangleFilter>::New();
    triangle->PassVertsOff();
    triangle->PassLinesOff();
    triangle->SetInputData(poly_data);

    auto clean = vtkSmartPointer<vtkCleanPolyData>::New();
    clean->SetInputConnection(triangle->GetOutputPort());
    clean->ToleranceIsAbsoluteOn();
    clean->SetAbsoluteTolerance(0.0);
    clean->PointMergingOn();

    auto remove_duplicate_polys = vtkSmartPointer<vtkRemoveDuplicatePolys>::New();
    remove_duplicate_polys->SetInputConnection(clean->GetOutputPort());

    auto sampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
    sampler->SetInputConnection(remove_duplicate_polys->GetOutputPort());
    sampler->SetPointGenerationModeToRegular();
    sampler->GenerateVertexPointsOff();
    sampler->GenerateEdgePointsOn();
    sampler->GenerateInteriorPointsOn();
    sampler->GenerateVerticesOff();
    sampler->SetDistance(effective_distance);
    sampler->Update();

    auto* surface = remove_duplicate_polys->GetOutput();
    if (surface == nullptr) {
        return std::unexpected { "Failed to prepare model surface" };
    }

    auto sampled = sampler->GetOutput();
    auto* points = sampled->GetPoints();

    if (points == nullptr || points->GetNumberOfPoints() == 0) {
        return std::unexpected { "Failed to sample pointcloud from model" };
    }

    auto raw_samples = std::vector<Position3> { };
    raw_samples.reserve(points->GetNumberOfPoints());

    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
        auto point = std::array<double, 3> { };
        points->GetPoint(i, point.data());
        raw_samples.push_back(point);
    }

    deduplicate_overlapped_samples(raw_samples, effective_distance * 0.45);

    const auto surface_diagonal = calculate_bounds_diagonal(surface);
    const auto probe_offset =
        std::max(effective_distance * 0.4, std::max(surface_diagonal * 1e-5, 1e-6));
    remove_internal_surface_samples(raw_samples, surface, probe_offset);

    regularize_density(raw_samples, effective_distance);

    auto positions = std::vector<std::tuple<double, double, double>> { };
    positions.reserve(raw_samples.size());

    for (auto const& point : raw_samples) {
        positions.emplace_back(point[0] * parameters.unit_scale, point[1] * parameters.unit_scale,
            point[2] * parameters.unit_scale);
    }

    apply_max_points_limit(positions, parameters.max_points);

    return positions;
}

}
