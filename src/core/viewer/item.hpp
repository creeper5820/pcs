#pragma once

#include "utility/common.hpp"

#include <cassert>
#include <memory>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace core::view {
struct Item {
public:
    using PointCloudT = pcl::PointCloud<pcl::PointXYZRGB>;

    struct RGB {
        uint8_t r;
        uint8_t g;
        uint8_t b;

        bool operator==(const RGB& rgb) const {
            return r == rgb.r
                && g == rgb.g
                && b == rgb.b;
        }
    };

    template <typename PointT>
    Item(pcl::PointCloud<PointT>::Ptr cloud) {
        assert(cloud != nullptr);
        cloud_ = std::make_shared<PointCloudT>();
        pcl::copyPointCloud(*cloud, *cloud_);
        loaded_ = true;
    };

    Item(const std::string& path)
        : path_(path) {
        cloud_ = std::make_shared<PointCloudT>();
        auto xyz = pcl::PointCloud<pcl::PointXYZ> {};
        if (pcl::io::loadPCDFile(path, xyz) == 0) {
            pcl::copyPointCloud(xyz, *cloud_);
            loaded_ = true;
        }
    }

    bool save(const std::string& path = {}) {
        if (path_.empty() && path.empty()) {
            util::message("save", "path is empty");
            return false;
        }
        auto& target = path.empty() ? path_ : path;
        if (!pcl::io::savePCDFileBinary(target, *cloud_)) {
            util::message("save", target);
            return true;
        } else {
            util::message("save", "failed");
            return false;
        }
    }

    void set_color(uint8_t r, uint8_t g, uint8_t b) {
        if (color_ == RGB { r, g, b })
            return;
        for (auto& point : cloud_->points) {
            point.r = r;
            point.g = g;
            point.b = b;
        }
        color_ = RGB { r, g, b };
    }

    bool loaded() { return loaded_; }
    auto path() { return path_; }
    auto frame() { return frame_; }
    auto color() { return color_; }
    auto size() { return cloud_->size(); }
    auto operator*() { return cloud_; }

private:
    PointCloudT::Ptr cloud_;
    std::string path_ { "nothing" };
    std::string frame_ { "default" };
    RGB color_ {};
    bool loaded_ { false };
};
}