#pragma once

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>

namespace core::cloud::algorithm {

template <typename PointT>
class Compress {
public:
    using PointCloud = pcl::PointCloud<PointT>;

    explicit Compress();
    ~Compress();

    Compress(const Compress&) = delete;
    Compress& operator=(const Compress&) = delete;

    Eigen::MatrixXf operator()(const PointCloud& cloud);

    Eigen::MatrixXf apply(const PointCloud& cloud);

    void setResolution(float resolution);

    void setRangeX(float min, float max);

    void setRangeY(float min, float max);

    void setRangeBlindX(float min, float max);

    void setRangeBlindY(float min, float max);

private:
    struct Impl;
    Impl* pimpl_;
};

}