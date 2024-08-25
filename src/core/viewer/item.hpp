#pragma once

#include "utility/common.hpp"

#include <cassert>
#include <memory>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace core::viewer {
struct Item {
public:
    using PointCloudT = pcl::PointCloud<pcl::PointXYZRGB>;

    template <typename PointT>
    Item(pcl::PointCloud<PointT>::Ptr cloud) {
        assert(cloud != nullptr);
        cloud_ = std::make_shared<PointCloudT>();
        pcl::copyPointCloud(*cloud, *cloud_);
        set_color({ 255, 255, 255 });
        loaded_ = true;
    };

    Item(const std::string& path)
        : path_(path) {
        cloud_ = std::make_shared<PointCloudT>();
        auto xyz = pcl::PointCloud<pcl::PointXYZ> {};
        if (pcl::io::loadPCDFile(path, xyz) == 0) {
            pcl::copyPointCloud(xyz, *cloud_);
            set_color({ 255, 255, 255 });
            loaded_ = true;
        }
    }

    bool save(const std::string& path = {}) {
        if (path_.empty() && path.empty()) {
            utility::message("save", "failed", "path is empty");
            return false;
        }
        auto& target = path.empty() ? path_ : path;
        if (!pcl::io::savePCDFileBinary(target, *cloud_)) {
            utility::message("save", target);
            return true;
        } else {
            utility::message("save", "failed");
            return false;
        }
    }

    void set_color(pcl::RGB rgb) {
        std::for_each(cloud_->points.begin(),
            cloud_->points.end(),
            [=](pcl::PointXYZRGB& point) {
                point.r = rgb.r;
                point.g = rgb.g;
                point.b = rgb.b;
            });
        color_ = rgb;
    }

    bool loaded() { return loaded_; }
    auto& path() { return path_; }
    auto size() { return cloud_->size(); }
    auto color() { return color_; }
    auto frame() { return frame_; }
    auto operator*() { return cloud_; }

private:
    PointCloudT::Ptr cloud_;
    std::string path_;
    std::string frame_ { "default" };
    pcl::RGB color_ { 255, 255, 255 };
    bool loaded_ { false };
};
}