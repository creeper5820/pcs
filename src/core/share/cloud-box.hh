#pragma once
#include "core/share/common.hh"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace core::cloud {
struct CloudSource {
public:
    template <typename PointT>
    CloudSource(std::shared_ptr<pcl::PointCloud<PointT>> cloud) {
        if (cloud == nullptr) {
            loaded_ = false;
        } else {
            cloud_ = std::make_shared<core::RenderCloud>();
            pcl::copyPointCloud(*cloud, *cloud_);
            loaded_ = true;
        }
    };

    CloudSource(const std::string& path)
        : filePath_(path) {
        cloud_ = std::make_shared<core::RenderCloud>();
        auto status = pcl::io::loadPCDFile(path, *cloud_);
        loaded_ = status ? true : false;
    }

    CloudSource(const CloudSource&) = delete;
    CloudSource& operator=(const CloudSource&) = delete;

    bool save(const std::string& path) {
        if (!pcl::io::savePCDFileBinary(path, *cloud_))
            return true;
        return false;
    }

    auto loaded() const { return loaded_; }
    auto path() const { return filePath_; }
    auto size() const { return cloud_->size(); }
    auto points() const { return *cloud_; }
    auto operator*() { return cloud_; }

private:
    std::shared_ptr<core::RenderCloud> cloud_;
    std::string filePath_ { "/path/to/Alliance" };
    bool loaded_ { false };
};
}

using CloudBox = core::cloud::CloudSource;
