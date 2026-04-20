#pragma once

#include "core/assets.hh"

#include <expected>
#include <memory>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace pcs::event {

using PointcloudPosition = std::tuple<double, double, double>;

struct ApplyPointcloudAssetPoints {
    using Result = std::expected<void, std::string>;

    struct Meta {
        std::string_view name = "Apply Pointcloud Asset Points";
        bool recordable       = true;
        bool redoable         = true;
        bool main_thread      = true;
    };

    Meta meta { };
    AssetsManager* assets = nullptr;
    std::string asset_id;
    std::vector<PointcloudPosition> before_points;
    std::vector<PointcloudPosition> after_points;

    auto exec() noexcept -> Result;
    auto undo() noexcept -> Result;
    auto redo() noexcept -> Result;
};

struct PointcloudDuplicateSharedState {
    std::string source_asset_id;
    std::string duplicated_asset_id;
    std::string duplicated_name;
    std::vector<PointcloudPosition> points;
};

struct DuplicatePointcloudAsset {
    using Result = std::expected<std::string, std::string>;

    struct Meta {
        std::string_view name = "Duplicate Pointcloud Asset";
        bool recordable       = true;
        bool redoable         = true;
        bool main_thread      = true;
    };

    Meta meta { };
    AssetsManager* assets = nullptr;
    std::shared_ptr<PointcloudDuplicateSharedState> state;

    auto exec() noexcept -> Result;
    auto undo() noexcept -> std::expected<void, std::string>;
    auto redo() noexcept -> Result;
};

}
