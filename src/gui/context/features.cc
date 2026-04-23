#include "gui/context/features.hh"

#include "core/handle/asset-details-registration.hh"

#include "core/events/render/model.hh"
#include "core/events/render/points.hh"
#include "gui/interaction/picker-mode.hh"
#include "gui/interaction/picker-png-map-extension.hh"
#include "gui/interaction/picker-pointcloud-extension.hh"
#include "gui/interaction/png-edit-mode.hh"
#include "gui/interaction/png-origin-pick-mode.hh"
#include "utility/morandi-pointcloud-color.hh"
#include "gui/working/panels/model-panel.hh"
#include "gui/working/panels/png-map-panel.hh"
#include "gui/working/panels/pointcloud-panel.hh"

#include <filesystem>
#include <memory>

namespace pcs::gui::context {

namespace {

    auto inferred_name(std::string const& path, std::string_view fallback) -> std::string {
        auto name = std::filesystem::path(path).filename().string();
        if (name.empty()) {
            return std::string { fallback };
        }
        return name;
    }

}

auto register_default_features(AppModules& modules) noexcept -> void {
    auto picker_mode =
        std::make_unique<gui::interaction::PickerMode>(*modules.renderer, *modules.assets);
    picker_mode->register_extension(gui::interaction::make_pointcloud_picker_extension());
    picker_mode->register_extension(gui::interaction::make_png_map_picker_extension());

    modules.mouse->register_mode(std::move(picker_mode));
    modules.mouse->register_mode(
        std::make_unique<gui::interaction::PngEditMode>(
            *modules.renderer, *modules.assets, *modules.runtime));
    modules.mouse->register_mode(
        std::make_unique<gui::interaction::PngOriginPickMode>(*modules.renderer, *modules.assets));

    modules.action_panels->register_factory(PointsHandle::kKind,
        [](gui::working::ActionPanelContext context, QFont const& font) {
            return std::make_unique<gui::working::PointcloudPanel>(std::move(context), font);
        });
    modules.action_panels->register_factory(
        ModelHandle::kKind,
        [](gui::working::ActionPanelContext context, QFont const& font) {
            return std::make_unique<gui::working::ModelPanel>(std::move(context), font);
        });
    modules.action_panels->register_factory(
        PngMapHandle::kKind,
        [](gui::working::ActionPanelContext context, QFont const& font) {
            return std::make_unique<gui::working::PngMapPanel>(std::move(context), font);
        });

    pcs::handle::register_all_asset_details_providers(*modules.asset_details);

    modules.open_control->register_format({
        .label          = "点云文件",
        .dialog_pattern = "*.pcd",
        .extensions     = { ".pcd" },
        .open           = [&modules](std::string const& path) -> std::expected<void, std::string> {
            auto event = pcs::event::MakePointsUnit { };
            event.path = path;
            event.name = inferred_name(path, "pointcloud.pcd");

            auto result = modules.runtime->submit(std::move(event)).get();
            if (!result.has_value()) {
                return std::unexpected { result.error() };
            }

            auto id = modules.assets->register_asset<PointsHandle>(
                std::move(result.value()), inferred_name(path, "pointcloud.pcd"), path, true);
            if (auto handle = modules.assets->get_handle<PointsHandle>(id);
                handle.has_value() && handle.value() != nullptr) {
                const auto color = utility::next_morandi_pointcloud_color();
                handle.value()->set_overall_color(color.r, color.g, color.b);
                modules.assets->update_renderer();
            }
            return { };
        },
    });
    modules.open_control->register_format({
        .label          = "模型文件",
        .dialog_pattern = "*.obj",
        .extensions     = { ".obj" },
        .open           = [&modules](std::string const& path) -> std::expected<void, std::string> {
            auto event = pcs::event::MakeModelUnit { };
            event.path = path;
            event.name = inferred_name(path, "model.obj");

            auto result = modules.runtime->submit(std::move(event)).get();
            if (!result.has_value()) {
                return std::unexpected { result.error() };
            }

            modules.assets->register_asset<ModelHandle>(
                std::move(result.value()), inferred_name(path, "model.obj"), path);
            return { };
        },
    });
}

}
