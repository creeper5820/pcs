#include "pointcloud-assets.hh"

#include <utility>

namespace pcs::event {

auto ApplyPointcloudAssetPoints::exec() noexcept -> Result {
    if (assets == nullptr) {
        return std::unexpected { "点云资产管理器不可用" };
    }
    if (asset_id.empty()) {
        return std::unexpected { "点云资产 ID 为空" };
    }

    auto result = assets->replace_pointcloud_asset_data(asset_id, after_points);
    if (!result.has_value()) {
        return std::unexpected { result.error() };
    }

    assets->update_renderer();
    return { };
}

auto ApplyPointcloudAssetPoints::undo() noexcept -> Result {
    if (assets == nullptr) {
        return std::unexpected { "点云资产管理器不可用" };
    }
    if (asset_id.empty()) {
        return std::unexpected { "点云资产 ID 为空" };
    }

    auto result = assets->replace_pointcloud_asset_data(asset_id, before_points);
    if (!result.has_value()) {
        return std::unexpected { result.error() };
    }

    assets->update_renderer();
    return { };
}

auto ApplyPointcloudAssetPoints::redo() noexcept -> Result { return exec(); }

auto DuplicatePointcloudAsset::exec() noexcept -> Result {
    if (assets == nullptr || state == nullptr) {
        return std::unexpected { "点云资产管理器不可用" };
    }
    if (state->source_asset_id.empty()) {
        return std::unexpected { "源点云资产 ID 为空" };
    }

    if (state->points.empty()) {
        auto handle_result = assets->get_pointcloud_handle(state->source_asset_id);
        if (!handle_result.has_value() || handle_result.value() == nullptr) {
            return std::unexpected { "源点云资产不可用" };
        }

        state->points = handle_result.value()->get_positions();
    }

    auto pointcloud = std::make_unique<PointsHandle>();
    auto load_result = pointcloud->load_from_positions(state->points);
    if (!load_result.has_value()) {
        return std::unexpected { std::string { load_result.error() } };
    }

    auto name = state->duplicated_name;
    if (name.empty()) {
        auto source_name = assets->get_asset_name(state->source_asset_id).value_or("pointcloud");
        name             = source_name + "-copy.pcd";
        state->duplicated_name = name;
    }

    state->duplicated_asset_id =
        assets->register_pointcloud_asset(std::move(pointcloud), name, { }, false);
    return state->duplicated_asset_id;
}

auto DuplicatePointcloudAsset::undo() noexcept -> std::expected<void, std::string> {
    if (assets == nullptr || state == nullptr) {
        return std::unexpected { "点云资产管理器不可用" };
    }
    if (state->duplicated_asset_id.empty()) {
        return std::unexpected { "复制资产 ID 不可用" };
    }

    if (!assets->remove_asset(state->duplicated_asset_id)) {
        return std::unexpected { "删除复制点云资产失败" };
    }

    return { };
}

auto DuplicatePointcloudAsset::redo() noexcept -> Result { return exec(); }

}
