#include "core/assets.hh"
#include "core/handle/points.hh"
#include "gui/working/asset-details.hh"

#include <filesystem>
#include <QString>

namespace pcs::handle::points {

namespace {

auto asset_size_text(std::string const& path, std::uintmax_t estimated_bytes) -> QString {
    if (path.empty() || path == "<memory>") {
        const auto kb = estimated_bytes / 1024;
        const auto mb = kb / 1024;
        if (mb > 0) {
            return QString("约 %1 MB (内存)").arg(mb);
        }
        return QString("约 %1 KB (内存)").arg(kb);
    }

    auto error = std::error_code { };
    const auto size = std::filesystem::file_size(path, error);
    if (error) {
        return "未知大小";
    }

    const auto kb = size / 1024;
    const auto mb = kb / 1024;
    if (mb > 0) {
        return QString("%1 MB").arg(mb);
    }
    return QString("%1 KB").arg(kb);
}

}

auto register_details_provider(gui::working::AssetDetailsRegistry& registry) noexcept -> void {
    registry.register_provider(std::type_index { typeid(PointsHandle) },
        [](AssetsManager& assets,
            std::string const& id) -> std::optional<gui::working::AssetDetails> {
            auto result = assets.get_handle<PointsHandle>(id);
            if (!result.has_value()) {
                return std::nullopt;
            }

            auto* handle               = result.value();
            const auto points_count    = static_cast<std::uintmax_t>(handle->get_points_size());
            const auto path            = assets.get_asset_path(id);
            const auto estimated_bytes = points_count * 3U * sizeof(float);
            const auto is_memory       = path == "<memory>";

            return gui::working::AssetDetails {
                .type = "点云",
                .size = asset_size_text(path, estimated_bytes),
                .info = QString("点数: %1 点, 状态: %2")
                    .arg(static_cast<qulonglong>(points_count))
                    .arg(is_memory ? "内存中" : "已保存"),
            };
        });
}

}
