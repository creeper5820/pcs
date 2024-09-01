#pragma once

#include "utility/common.hh"

#include <cassert>
#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace core::cloud {
struct Item {
public:
    using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

    struct RGB {
        double r;
        double g;
        double b;

        bool operator==(const RGB& rgb) const {
            return r == rgb.r && g == rgb.g && b == rgb.b;
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
        if (pcl::io::loadPCDFile(path, *cloud_) == 0) {
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

    void setColor(double r, double g, double b) {
        color_ = { r, g, b };
    }

    void setVisible(bool flag) {
        visible_ = flag;
    }

    auto loaded() { return loaded_; }
    auto visible() { return visible_; }
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
    bool visible_ { false };
};
}