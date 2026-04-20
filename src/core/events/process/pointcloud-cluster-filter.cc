#include "pointcloud-cluster-filter.hh"

#include <algorithm>
#include <cmath>
#include <queue>

namespace pcs::event {

namespace {

auto distance_sq(PointcloudClusterFilter::Position const& a,
    PointcloudClusterFilter::Position const& b) noexcept -> double {
    const auto dx = std::get<0>(a) - std::get<0>(b);
    const auto dy = std::get<1>(a) - std::get<1>(b);
    const auto dz = std::get<2>(a) - std::get<2>(b);
    return dx * dx + dy * dy + dz * dz;
}

}

auto PointcloudClusterFilter::exec() noexcept -> Result {
    if (points.empty()) {
        return std::unexpected { "点云数据为空" };
    }
    if (tolerance <= 0.0) {
        return std::unexpected { "聚类距离必须大于 0" };
    }
    if (min_cluster_size == 0) {
        return std::unexpected { "最小簇点数必须大于 0" };
    }

    const auto eps2 = tolerance * tolerance;
    auto visited    = std::vector<bool>(points.size(), false);
    auto clusters   = std::vector<std::vector<std::size_t>> { };

    for (std::size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) {
            continue;
        }

        visited[i] = true;
        auto queue = std::queue<std::size_t> { };
        queue.push(i);

        auto indices = std::vector<std::size_t> { i };
        while (!queue.empty()) {
            const auto current = queue.front();
            queue.pop();

            for (std::size_t j = 0; j < points.size(); ++j) {
                if (visited[j]) {
                    continue;
                }
                if (distance_sq(points[current], points[j]) > eps2) {
                    continue;
                }

                visited[j] = true;
                queue.push(j);
                indices.push_back(j);
            }
        }

        clusters.push_back(std::move(indices));
    }

    auto filtered = std::vector<Position> { };
    if (mode == PointcloudClusterMode::KeepLargestCluster) {
        auto iter = std::max_element(clusters.begin(), clusters.end(),
            [](auto const& lhs, auto const& rhs) { return lhs.size() < rhs.size(); });
        if (iter == clusters.end()) {
            return std::unexpected { "未找到有效聚类" };
        }

        filtered.reserve(iter->size());
        for (const auto index : *iter) {
            filtered.push_back(points[index]);
        }
    } else {
        for (auto const& cluster : clusters) {
            if (cluster.size() < min_cluster_size) {
                continue;
            }
            for (const auto index : cluster) {
                filtered.push_back(points[index]);
            }
        }
    }

    if (filtered.empty()) {
        return std::unexpected { "过滤后点云为空" };
    }

    return filtered;
}

auto PointcloudClusterFilter::redo() noexcept -> Result { return exec(); }

}
