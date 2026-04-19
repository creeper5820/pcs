#include "gui/context/features.hh"

#include "core/map/png-map-transform.hh"

#include "core/events/render/model.hh"
#include "core/events/render/points.hh"
#include "gui/interaction/picker-mode.hh"
#include "gui/interaction/picker-png-map-extension.hh"
#include "gui/interaction/picker-pointcloud-extension.hh"
#include "gui/interaction/png-edit-mode.hh"
#include "gui/interaction/png-origin-pick-mode.hh"
#include "gui/working/panels/model-panel.hh"
#include "gui/working/panels/png-map-panel.hh"
#include "gui/working/panels/pointcloud-panel.hh"

#include <filesystem>

namespace pcs::gui::context {

namespace {

    auto inferred_name(std::string const& path, std::string_view fallback) -> std::string {
        auto name = std::filesystem::path(path).filename().string();
        if (name.empty()) {
            return std::string { fallback };
        }
        return name;
    }

    auto trim_decimal_zeros(QString text) noexcept -> QString {
        while (text.contains('.') && text.endsWith('0')) {
            text.chop(1);
        }
        if (text.endsWith('.')) {
            text.chop(1);
        }
        if (text == "-0") {
            return "0";
        }
        return text;
    }

    auto format_size_mb(std::uintmax_t bytes) noexcept -> QString {
        const auto mb = static_cast<double>(bytes) / (1024.0 * 1024.0);
        return trim_decimal_zeros(QString::number(mb, 'f', 2));
    }

    auto asset_size_mb_text(std::string const& path, std::uintmax_t fallback_bytes) noexcept
        -> QString {
        if (!path.empty() && path != "<memory>") {
            auto error      = std::error_code { };
            const auto size = std::filesystem::file_size(path, error);
            if (!error) {
                return format_size_mb(size);
            }
        }

        return format_size_mb(fallback_bytes);
    }

    auto export_mirror_text(pcs::PngMapExportMirror mirror) noexcept -> QString {
        switch (mirror) {
        case pcs::PngMapExportMirror::Horizontal:
            return "左右镜像";
        case pcs::PngMapExportMirror::Vertical:
            return "上下镜像";
        case pcs::PngMapExportMirror::None:
            return "不镜像";
        }

        return "左右镜像";
    }

}

auto register_default_features(AppModules& modules) noexcept -> void {
    auto picker_mode =
        std::make_unique<gui::interaction::PickerMode>(*modules.renderer, *modules.assets);
    picker_mode->register_extension(gui::interaction::make_pointcloud_picker_extension());
    picker_mode->register_extension(gui::interaction::make_png_map_picker_extension());

    modules.mouse->register_mode(std::move(picker_mode));
    modules.mouse->register_mode(
        std::make_unique<gui::interaction::PngEditMode>(*modules.renderer, *modules.assets));
    modules.mouse->register_mode(
        std::make_unique<gui::interaction::PngOriginPickMode>(*modules.renderer, *modules.assets));

    modules.action_panels->register_factory(pcs::AssetKind::Pointcloud,
        [](gui::working::ActionPanelContext context, QFont const& font) {
            return gui::working::make_pointcloud_panel(std::move(context), font);
        });
    modules.action_panels->register_factory(
        pcs::AssetKind::Model, [](gui::working::ActionPanelContext context, QFont const& font) {
            return gui::working::make_model_panel(std::move(context), font);
        });
    modules.action_panels->register_factory(
        pcs::AssetKind::PngMap, [](gui::working::ActionPanelContext context, QFont const& font) {
            return gui::working::make_png_map_panel(std::move(context), font);
        });

    modules.asset_details->register_provider(pcs::AssetKind::Pointcloud,
        [](pcs::AssetsManager& assets,
            std::string const& id) -> std::optional<pcs::gui::working::AssetDetails> {
            auto result = assets.get_pointcloud_handle(id);
            if (!result.has_value()) {
                return std::nullopt;
            }

            auto* handle               = result.value();
            const auto points_count    = static_cast<std::uintmax_t>(handle->get_points_size());
            const auto path            = assets.get_asset_path(id).value_or(std::string { });
            const auto estimated_bytes = points_count * 3U * sizeof(float);
            const auto is_memory       = path == "<memory>";

            return pcs::gui::working::AssetDetails {
                .type = "点云",
                .size = asset_size_mb_text(path, estimated_bytes),
                .info = QString("点数: %1 点, 状态: %2")
                    .arg(static_cast<qulonglong>(points_count))
                    .arg(is_memory ? "内存中" : "已保存"),
            };
        });
    modules.asset_details->register_provider(pcs::AssetKind::Model,
        [](pcs::AssetsManager& assets,
            std::string const& id) -> std::optional<pcs::gui::working::AssetDetails> {
            auto result = assets.get_model_handle(id);
            if (!result.has_value()) {
                return std::nullopt;
            }

            auto* handle        = result.value();
            const auto vertices = static_cast<std::uintmax_t>(handle->get_points_size());
            const auto faces    = static_cast<std::uintmax_t>(handle->get_polys_size());
            const auto path     = assets.get_asset_path(id).value_or(std::string { });
            const auto estimated_bytes =
                vertices * 3U * sizeof(float) + faces * 3U * sizeof(std::uint32_t);

            return pcs::gui::working::AssetDetails {
                .type = "模型",
                .size = asset_size_mb_text(path, estimated_bytes),
                .info = QString("顶点: %1 点, 面数: %2 面")
                    .arg(static_cast<qulonglong>(vertices))
                    .arg(static_cast<qulonglong>(faces)),
            };
        });
    modules.asset_details->register_provider(pcs::AssetKind::PngMap,
        [](pcs::AssetsManager& assets,
            std::string const& id) -> std::optional<pcs::gui::working::AssetDetails> {
            auto result = assets.get_png_map_handle(id);
            if (!result.has_value()) {
                return std::nullopt;
            }

            auto* handle           = result.value();
            const auto width       = static_cast<std::uintmax_t>(handle->get_width());
            const auto height      = static_cast<std::uintmax_t>(handle->get_height());
            const auto path        = assets.get_asset_path(id).value_or(std::string { });
            const auto config      = handle->get_frame_config();
            const auto origin_text = [&] {
                if (config.origin_pixel_x.has_value() && config.origin_pixel_y.has_value()) {
                    return QString("(%1, %2)")
                        .arg(*config.origin_pixel_x)
                        .arg(*config.origin_pixel_y);
                }
                return QString("中心");
            }();

            return pcs::gui::working::AssetDetails {
                .type = "PNG 地图",
                .size = asset_size_mb_text(path, width * height),
                .info = QString("尺寸: %1 x %2 像素, 分辨率: %3 米/像素, Yaw: %4, 原点: %5, 导出: %6")
                    .arg(static_cast<qulonglong>(width))
                    .arg(static_cast<qulonglong>(height))
                    .arg(handle->get_resolution(), 0, 'f', 3)
                    .arg(config.yaw_deg, 0, 'f', 3)
                    .arg(origin_text)
                    .arg(export_mirror_text(config.export_mirror)),
            };
        });

    modules.open_control->register_format({
        .label          = "点云文件",
        .dialog_pattern = "*.pcd",
        .extensions     = { ".pcd" },
        .open           = [&modules](std::string const& path) -> std::expected<void, std::string> {
            auto context  = std::make_unique<pcs::event::MakePointsUnit::Context>();
            context->path = path;
            context->name = inferred_name(path, "pointcloud.pcd");

            auto result = pcs::event::MakePointsUnit::runtime_exec(std::move(context));
            if (!result.has_value()) {
                return std::unexpected { result.error() };
            }

            modules.assets->register_pointcloud_asset(
                std::move(result.value()), inferred_name(path, "pointcloud.pcd"), path, true);
            return { };
        },
    });
    modules.open_control->register_format({
        .label          = "模型文件",
        .dialog_pattern = "*.obj",
        .extensions     = { ".obj" },
        .open           = [&modules](std::string const& path) -> std::expected<void, std::string> {
            auto context  = std::make_unique<pcs::event::MakeModelUnit::Context>();
            context->path = path;
            context->name = inferred_name(path, "model.obj");

            auto result = pcs::event::MakeModelUnit::runtime_exec(std::move(context));
            if (!result.has_value()) {
                return std::unexpected { result.error() };
            }

            modules.assets->register_model_asset(
                std::move(result.value()), inferred_name(path, "model.obj"), path);
            return { };
        },
    });
}

}
