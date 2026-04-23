#include "core/assets.hh"
#include "core/handle/model.hh"
#include "gui/working/asset-details.hh"

#include <filesystem>
#include <QString>

namespace pcs::handle::model {

namespace {

auto asset_size_text(std::string const& path) -> QString {
    if (path.empty() || path == "<memory>") {
        return "内存中";
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
    registry.register_provider(std::type_index { typeid(ModelHandle) },
        [](AssetsManager& assets,
            std::string const& id) -> std::optional<gui::working::AssetDetails> {
            auto result = assets.get_handle<ModelHandle>(id);
            if (!result.has_value()) {
                return std::nullopt;
            }

            auto* handle            = result.value();
            const auto points_count = static_cast<std::uintmax_t>(handle->get_points_size());
            const auto polys_count  = static_cast<std::uintmax_t>(handle->get_polys_size());
            const auto path         = assets.get_asset_path(id);

            return gui::working::AssetDetails {
                .type = "模型",
                .size = asset_size_text(path),
                .info = QString("顶点: %1, 面片: %2")
                    .arg(static_cast<qulonglong>(points_count))
                    .arg(static_cast<qulonglong>(polys_count)),
            };
        });
}

}
