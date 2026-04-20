#pragma once

#include <expected>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace pcs::event {

enum class PointcloudClusterMode {
    KeepLargestCluster,
    RemoveSmallClusters,
};

struct PointcloudClusterFilter {
    using Position = std::tuple<double, double, double>;
    using Result   = std::expected<std::vector<Position>, std::string>;

    struct Meta {
        std::string_view name = "Pointcloud Cluster Filter";
        bool recordable       = false;
        bool redoable         = true;
    };

    Meta meta { };
    std::vector<Position> points;
    double tolerance = 0.3;
    std::size_t min_cluster_size = 20;
    PointcloudClusterMode mode { PointcloudClusterMode::RemoveSmallClusters };

    auto exec() noexcept -> Result;
    auto redo() noexcept -> Result;
};

}
