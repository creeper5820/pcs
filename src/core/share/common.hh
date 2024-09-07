#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace core {
using RenderPoint = pcl::PointXYZ;
using RenderCloud = pcl::PointCloud<RenderPoint>;
using RenderColor = std::tuple<double, double, double>;
}