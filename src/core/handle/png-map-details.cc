#include "core/assets.hh"
#include "core/handle/png-map.hh"
#include "gui/working/asset-details.hh"

#include <filesystem>
#include <QString>

namespace pcs::handle::png_map {

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
    registry.register_provider(std::type_index { typeid(PngMapHandle) },
        [](AssetsManager& assets,
            std::string const& id) -> std::optional<gui::working::AssetDetails> {
            auto result = assets.get_handle<PngMapHandle>(id);
            if (!result.has_value()) {
                return std::nullopt;
            }

            auto* handle         = result.value();
            const auto width     = handle->get_width();
            const auto height    = handle->get_height();
            const auto resolution = handle->get_resolution();
            const auto path      = assets.get_asset_path(id);

            return gui::working::AssetDetails {
                .type = "PNG 地图",
                .size = asset_size_text(path, width * height * 4),
                .info = QString("尺寸: %1×%2, 分辨率: %3 m/px")
                    .arg(width)
                    .arg(height)
                    .arg(resolution, 0, 'f', 3),
            };
        });
}

}
