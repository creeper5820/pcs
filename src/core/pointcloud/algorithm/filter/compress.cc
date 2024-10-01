#include "compress.hh"
#include "utility/math.hh"

#include <pcl/point_types.h>

namespace core::cloud::algorithm {
template <typename PointT>
struct Compress<PointT>::Impl {
public:
    Eigen::MatrixXf apply(const PointCloud& cloud) {
        const size_t row = rangeX_.length() / resolution_;
        const size_t col = rangeY_.length() / resolution_;
        Eigen::MatrixXf cells { row, col };

        float centerX = rangeX_.mid();
        float centerY = rangeY_.mid();

        for (const auto& point : cloud) {
            if (!(rangeX_(point.x) && rangeY_(point.y)))
                continue;
            if (rangeBlindX_(point.x) && rangeBlindY_(point.y))
                continue;
            const size_t x = (point.x - centerX) / resolution_;
            const size_t y = (point.y - centerY) / resolution_;
            cells(x, y) = std::max(cells(x, y), point.z);
        }
        return cells;
    }

    void setResolution(float resolution) {
        resolution_ = resolution;
        assert(resolution_ > 0);
    }

    void setRangeX(float min, float max) {
        rangeX_ = util::Range<float> { min, max };
        assert(rangeX_.valid());
    }

    void setRangeY(float min, float max) {
        rangeY_ = util::Range<float> { min, max };
        assert(rangeY_.valid());
    }

    void setRangeBlindX(float min, float max) {
        rangeBlindX_ = util::Range<float> { min, max };
        assert(rangeBlindX_.valid());
    }

    void setRangeBlindY(float min, float max) {
        rangeBlindY_ = util::Range<float> { min, max };
        assert(rangeBlindY_.valid());
    }

private:
    float resolution_ = 0.1f;

    util::Range<float> rangeX_ { -5.0f, 5.0f };
    util::Range<float> rangeY_ { -5.0f, 5.0f };

    util::Range<float> rangeBlindX_ { -0.3f, 0.3f };
    util::Range<float> rangeBlindY_ { -0.3f, 0.3f };
};

template <typename PointT>
Compress<PointT>::Compress()
    : pimpl_(new Impl) {
}

template <typename PointT>
Compress<PointT>::~Compress() {
    delete pimpl_;
}

template <typename PointT>
Eigen::MatrixXf Compress<PointT>::operator()(const PointCloud& cloud) {
    return pimpl_->apply(cloud);
}

template <typename PointT>
Eigen::MatrixXf Compress<PointT>::apply(const PointCloud& cloud) {
    return pimpl_->apply(cloud);
}

template <typename PointT>
void Compress<PointT>::setResolution(float resolution) {
    pimpl_->setResolution(resolution);
}

template <typename PointT>
void Compress<PointT>::setRangeX(float min, float max) {
    pimpl_->setRangeX(min, max);
}

template <typename PointT>
void Compress<PointT>::setRangeY(float min, float max) {
    pimpl_->setRangeY(min, max);
}

template <typename PointT>
void Compress<PointT>::setRangeBlindX(float min, float max) {
    pimpl_->setRangeBlindX(min, max);
}

template <typename PointT>
void Compress<PointT>::setRangeBlindY(float min, float max) {
    pimpl_->setRangeBlindY(min, max);
}

template class Compress<pcl::PointXYZ>;
template class Compress<pcl::PointXYZRGB>;
template class Compress<pcl::PointNormal>;
}