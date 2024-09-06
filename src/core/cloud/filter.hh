#pragma once
#include "item.hh"
#include "utility/single.hh"

namespace core::cloud {
class Filter : public util::Singleton<Filter> {
public:
    Filter(util::Singleton<Filter>::token);
    ~Filter();
    Filter(const Filter&) = delete;
    Filter& operator=(const Filter&) = delete;

    /// Apply
    void apply(PointCloud& pointCloud);
    void extract(const PointCloud& input, PointCloud& output);

    /// Select
    int addSelectPointArea(Eigen::Vector3d point);
    int addSelectCubeArea(Eigen::Vector3d corner[2]);
    int addSelectCubeArea(Eigen::Vector3d center, double length);
    int addSelectSphereArea(Eigen::Vector3d center, double radius);

    void removeAllSelectArea();
    void removeSelectArea(int index);

private:
    struct Impl;
    Impl* pimpl_;
};
}