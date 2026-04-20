#include "pointcloud-cluster-filter.hh"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pcs::event {

namespace {

using Point      = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

auto as_pointcloud(std::vector<PointcloudClusterFilter::Position> const& points)
    -> pcl::PointCloud<Point>::Ptr {
    auto cloud = pcl::make_shared<PointCloud>();
    cloud->points.reserve(points.size());

    for (auto const& [x, y, z] : points) {
        cloud->points.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    }

    cloud->width  = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    return cloud;
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

    auto cloud = as_pointcloud(points);
    auto tree  = pcl::make_shared<pcl::search::KdTree<Point>>();
    tree->setInputCloud(cloud);

    auto cluster_indices = std::vector<pcl::PointIndices> { };
    auto extractor       = pcl::EuclideanClusterExtraction<Point> { };
    extractor.setSearchMethod(tree);
    extractor.setInputCloud(cloud);
    extractor.setClusterTolerance(static_cast<float>(tolerance));
    extractor.setMinClusterSize(1);
    extractor.setMaxClusterSize(static_cast<int>(points.size()));
    extractor.extract(cluster_indices);

    if (cluster_indices.empty()) {
        return std::unexpected { "未找到有效聚类" };
    }

    auto filtered = std::vector<Position> { };
    if (mode == PointcloudClusterMode::KeepLargestCluster) {
        auto iter = std::max_element(cluster_indices.begin(), cluster_indices.end(),
            [](auto const& lhs, auto const& rhs) { return lhs.indices.size() < rhs.indices.size(); });

        filtered.reserve(iter->indices.size());
        for (const auto index : iter->indices) {
            filtered.push_back(points[index]);
        }
    } else {
        for (auto const& cluster : cluster_indices) {
            if (cluster.indices.size() < min_cluster_size) {
                continue;
            }
            for (const auto index : cluster.indices) {
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
