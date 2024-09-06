#include "filter.hh"

#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

using namespace core::cloud;

struct Filter::Impl {
public:
    using PointCloudT = PointCloud::PointCloudT;
    using PointT = PointCloud::PointT;

    Impl() = default;

    /// Apply
    void apply(PointCloud& pointCloud) { }

    void extract(const PointCloud& input, PointCloud& output) { }

    /// Select
    int addSelectPointArea(Eigen::Vector3d point) { }

    int addSelectCubeArea(Eigen::Vector3d corner[2]) { }

    int addSelectCubeArea(Eigen::Vector3d center, double length) { }

    int addSelectSphereArea(Eigen::Vector3d center, double radius) { }

    void removeAllSelectArea() { }

    void removeSelectArea(int index) { }

private:
    pcl::BoxClipper3D<PointT> boxClipper_ { Eigen::Affine3f::Identity() };
    pcl::RadiusOutlierRemoval<PointT> radiusOutlierRemoval_;
    pcl::PassThrough<PointT> passThrough_;
};

Filter::Filter(util::Singleton<Filter>::token)
    : pimpl_(new Impl) {
}

Filter::~Filter() {
    delete pimpl_;
}

/// Apply
void Filter::apply(PointCloud& pointCloud) { }

void Filter::extract(const PointCloud& input, PointCloud& output) { }

/// Select
int Filter::addSelectPointArea(Eigen::Vector3d point) { }

int Filter::addSelectCubeArea(Eigen::Vector3d corner[2]) { }

int Filter::addSelectCubeArea(Eigen::Vector3d center, double length) { }

int Filter::addSelectSphereArea(Eigen::Vector3d center, double radius) { }

void Filter::removeAllSelectArea() { }

void Filter::removeSelectArea(int index) { }