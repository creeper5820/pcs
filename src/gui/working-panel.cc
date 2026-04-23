#include "working-panel.hh"

#include "core/handle/model.hh"
#include "core/handle/png-map.hh"
#include "core/handle/points.hh"
#include "core/renderer.hh"
#include "gui/component/assets-view.hh"
#include "gui/working/action-panels.hh"
#include "gui/working/panels/common.hh"
#include "utility/morandi-pointcloud-color.hh"

#include <creeper-qt/layout/flow.hh>
#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/layout/scroll.hh>
#include <creeper-qt/utility/material-icon.hh>
#include <creeper-qt/utility/wrapper/mutable-value.hh>
#include <creeper-qt/widget/buttons/icon-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/cards/outlined-card.hh>
#include <creeper-qt/widget/text.hh>
#include <creeper-qt/widget/widget.hh>

#include <QFontMetrics>
#include <QPointer>
#include <QSizePolicy>
#include <QStringListModel>
#include <QTimer>
#include <QVBoxLayout>

#include <qfiledialog.h>
#include <qmessagebox.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <expected>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

using namespace creeper;

namespace {

constexpr auto kPanelWidthScale = 1.3;
constexpr auto kPanelMinWidth   = 416;
constexpr auto kPanelMaxWidth   = 728;

static auto open_asset_location(QString const& filter) noexcept
    -> std::expected<std::string, std::string_view> {
    const auto location = QFileDialog::getOpenFileName(nullptr, "打开资产", "", filter);

    if (location.isEmpty()) {
        return std::unexpected { "用户取消了文件选择" };
    }

    return location.toStdString();
}

static auto save_model_location(std::string const& suggested_name) noexcept
    -> std::expected<std::string, std::string_view> {
    auto filename = std::filesystem::path(suggested_name);
    if (filename.extension() != ".obj") {
        filename.replace_extension(".obj");
    }

    const auto location = QFileDialog::getSaveFileName(
        nullptr, "保存模型", QString::fromStdString(filename.string()), "模型文件 (*.obj)");

    if (location.isEmpty()) {
        return std::unexpected { "用户取消保存模型" };
    }

    return location.toStdString();
}

auto fold_text_for_panel(QString const& value, int wrap_after = 36) noexcept -> QString {
    if (value.size() <= wrap_after || wrap_after <= 0) {
        return value;
    }

    auto folded         = QString { };
    auto segment_length = 0;
    folded.reserve(value.size() + value.size() / wrap_after);

    for (const auto ch : value) {
        folded.push_back(ch);

        if (ch == '\n') {
            segment_length = 0;
            continue;
        }

        if (ch.isSpace() || ch == '/' || ch == '\\' || ch == '-' || ch == '_' || ch == '.') {
            segment_length = 0;
            continue;
        }

        ++segment_length;
        if (segment_length >= wrap_after) {
            folded.push_back('\n');
            segment_length = 0;
        }
    }

    return folded;
}

}

struct WorkingPanel::Impl {
    WorkingPanel& self;
    ThemeManager& manager;
    pcs::AssetsManager& assets;
    pcs::Runtime& runtime;
    pcs::Renderer& renderer;
    pcs::gui::interaction::Mouse& mouse;
    pcs::gui::working::OpenControl& open_control;
    pcs::gui::working::AssetDetailsRegistry& asset_details_registry;
    pcs::gui::working::ActionPanelRegistry& action_panel_registry;
    MutableDouble& panel_width;
    bool& assets_visibility;

    theme::pro::ThemeManager theme_manager;
    QFont font { "WenQuanYi Micro Hei Mono", 10 };
    QStringListModel* location_list = nullptr;

    std::string current_asset_id;

    MutableQString asset_name { "未知" };
    MutableQString asset_type { "未知" };
    MutableQString asset_path { "未知" };
    MutableQString asset_size { "未知" };
    MutableQString asset_info { "未知" };

    QPointer<IconButton> visibility_button;
    QPointer<IconButton> delete_button;
    QPointer<IconButton> duplicate_asset_button;
    QPointer<IconButton> save_asset_button;
    QPointer<IconButton> save_as_asset_button;
    QPointer<AssetsView> assets_view;
    QPointer<Text> visibility_toggle_text;

    std::unique_ptr<pcs::gui::working::ActionPanelHost> action_host;

    Impl(WorkingPanel& self, ThemeManager& manager, pcs::AssetsManager& assets,
        pcs::Runtime& runtime, pcs::Renderer& renderer,
        pcs::gui::working::OpenControl& open_control,
        pcs::gui::working::AssetDetailsRegistry& asset_details_registry,
        pcs::gui::interaction::Mouse& mouse,
        pcs::gui::working::ActionPanelRegistry& action_panel_registry, MutableDouble& panel_width,
        bool& assets_visibility) noexcept
        : self { self }
        , manager { manager }
        , assets { assets }
        , runtime { runtime }
        , renderer { renderer }
        , mouse { mouse }
        , open_control { open_control }
        , asset_details_registry { asset_details_registry }
        , action_panel_registry { action_panel_registry }
        , panel_width { panel_width }
        , assets_visibility { assets_visibility }
        , theme_manager { manager } {
        location_list = new QStringListModel { &self };

        auto panel_context                = pcs::gui::working::ActionPanelContext { };
        panel_context.manager             = &manager;
        panel_context.assets              = &assets;
        panel_context.runtime             = &runtime;
        panel_context.renderer            = &renderer;
        panel_context.mouse               = &mouse;
        panel_context.refresh_assets_list = [this] { refresh_assets_list(); };
        panel_context.select_asset        = [this](std::string const& id) { select_asset(id); };

        action_host = std::make_unique<pcs::gui::working::ActionPanelHost>(
            panel_context, &action_panel_registry, font);

        sync_visibility_toggle_label();

        auto* layout = new QVBoxLayout { };
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(0);
        layout->addWidget(build_card());
        self.setLayout(layout);

        refresh_assets_list();
        clear_asset_detail();
    }

    auto build_card() noexcept -> QWidget* {
        return new FilledCard {
            card::pro::ThemeManager { manager },
            widget::pro::MinimumWidth { kPanelMinWidth },
            widget::pro::MaximumWidth { kPanelMaxWidth },
            MutableTransform {
                [](auto& widget, const auto& width) {
                    const auto scaled_width =
                        static_cast<int>(std::round(static_cast<double>(width) * kPanelWidthScale));
                    const auto clamped_width =
                        std::clamp(scaled_width, kPanelMinWidth, kPanelMaxWidth);
                    widget.setFixedWidth(clamped_width);
                },
                panel_width,
            },
            card::pro::Radius { 0 },
            card::pro::Layout<Col> {
                col::pro::Margin { 0 },
                col::pro::Item<FilledCard> {
                    card::pro::ThemeManager { manager },
                    card::pro::Radius { 10 },
                    card::pro::Layout<Col> {
                        col::pro::Margin { 0 },
                        col::pro::Spacing { 0 },
                        col::pro::Item<ScrollArea> {
                            theme_manager,
                            scroll::pro::ScrollBarPolicy {
                                Qt::ScrollBarAsNeeded, Qt::ScrollBarAlwaysOff },
                            scroll::pro::Item<Widget> {
                                widget::pro::Apply { [](QWidget& widget) {
                                    widget.setSizePolicy(
                                        QSizePolicy::Ignored, QSizePolicy::Preferred);
                                } },
                                widget::pro::Layout<Col> {
                                    col::pro::Margin { 10 },
                                    col::pro::Spacing { 10 },
                                    col::pro::Item<FilledCard> {
                                        card::pro::ThemeManager { manager },
                                        card::pro::Radius { 10 },
                                        card::pro::LevelHigh,
                                        card::pro::Layout<Col> {
                                            col::pro::Margin { 10 },
                                            col::pro::Spacing { 6 },
                                            col::pro::Item<Text> {
                                                text::pro::ThemeManager { manager },
                                                text::pro::Font { font },
                                                text::pro::Alignment { Qt::AlignHCenter },
                                                text::pro::Text { "资产操作" },
                                            },
                                            col::pro::Item<Row> {
                                                row::pro::Spacing { 10 },
                                                row::pro::Alignment { Qt::AlignLeft },
                                                row::pro::Item { make_assets_action("folder_open",
                                                    "打开", [this] { open_location(); }) },
                                                row::pro::Item { make_assets_action(
                                                    material::icon::kSave, "保存",
                                                    [this] { save_asset(); }, &save_asset_button) },
                                                row::pro::Item { make_assets_action(
                                                    material::icon::kFolderOpen, "另存",
                                                    [this] { save_asset_as(); },
                                                    &save_as_asset_button) },
                                                row::pro::Stretch { 255 },
                                            },
                                            col::pro::Item<Row> {
                                                row::pro::Spacing { 10 },
                                                row::pro::Alignment { Qt::AlignLeft },
                                                row::pro::Item { make_assets_action(
                                                    material::icon::kFileCopy, "复制",
                                                    [this] { duplicate_asset(); },
                                                    &duplicate_asset_button) },
                                                row::pro::Item { make_assets_action(
                                                    "hide_source", "隐藏全部",
                                                    [this] { hide_assets(); }, nullptr,
                                                    &visibility_toggle_text) },
                                                row::pro::Item { make_assets_action("delete_sweep",
                                                    "清空", [this] { clean_assets(); }) },
                                                row::pro::Item { make_assets_action("restart_alt",
                                                    "重置视角", [this] { reset_view(); }) },
                                                row::pro::Stretch { 255 },
                                            },
                                        },
                                    },
                                    col::pro::Item { build_assets_view() },
                                    col::pro::Item<FilledCard> {
                                        theme_manager,
                                        card::pro::LevelLowest,
                                        card::pro::Layout<Col> {
                                            col::pro::Margin { 10 },
                                            col::pro::Spacing { 6 },
                                            col::pro::Item<OutlinedCard> {
                                                theme_manager,
                                                card::pro::Layout<Col> {
                                                    col::pro::Margin { 8 },
                                                    col::pro::Spacing { 6 },
                                                    col::pro::Item {
                                                        make_prop_row("名称:", asset_name) },
                                                    col::pro::Item {
                                                        make_prop_row("类型:", asset_type) },
                                                    col::pro::Item {
                                                        make_prop_row("大小:", asset_size) },
                                                    col::pro::Item {
                                                        make_prop_row("信息:", asset_info) },
                                                    col::pro::Item {
                                                        make_prop_row("路径:", asset_path) },
                                                },
                                            },
                                            col::pro::Item { build_asset_actions_row() },
                                            col::pro::Item { action_host->widget() },
                                            col::pro::Stretch { 255 },
                                        },
                                    },
                                    col::pro::Stretch { 255 },
                                },
                            },
                        },
                    },
                },
            },
        };
    }

    auto build_assets_view() noexcept -> QWidget* {
        assets_view = new AssetsView {
            manager,
            assets,
            *location_list,
            [this](std::string const& id) { on_asset_selected(id); },
            [this](std::string const& id) { toggle_asset_visibility(id); },
            [this](std::string const& id) { delete_asset(id); },
        };

        return assets_view;
    }

    auto detail_label_width() const noexcept -> int {
        constexpr auto detail_labels = std::array { "名称:", "类型:", "大小:", "信息:", "路径:" };
        const auto metrics           = QFontMetrics { font };
        auto width                   = 0;

        for (const auto* label : detail_labels) {
            width = std::max(width, metrics.horizontalAdvance(QString::fromUtf8(label)));
        }

        return width + 14;
    }

    auto make_prop_row(char const* name, MutableQString& prop) const noexcept -> Row* {
        return new Row {
            row::pro::Spacing { 8 },
            row::pro::Margin { 5 },
            row::pro::Item<Text> {
                { 0, Qt::AlignTop },
                theme_manager,
                text::pro::Text { name },
                text::pro::Font { font },
                text::pro::Alignment { Qt::AlignTop | Qt::AlignLeft },
                widget::pro::FixedWidth { detail_label_width() },
            },
            row::pro::Item<Text> {
                { 1, Qt::AlignTop },
                theme_manager,
                text::pro::Font { font },
                text::pro::WordWrap { true },
                text::pro::Alignment { Qt::AlignTop | Qt::AlignLeft },
                widget::pro::Apply { [](QWidget& widget) {
                    widget.setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
                } },
                MutableTransform {
                    [](Text& text, QString const& value) {
                        text.setText(fold_text_for_panel(value));
                        text.setToolTip(value);
                    },
                    prop,
                },
            },
        };
    }

    auto build_asset_actions_row() noexcept -> Row* {
        visibility_button = new IconButton {
            icon_button::pro::ThemeManager { manager },
            icon_button::pro::FixedSize { IconButton::kSmallContainerSize },
            icon_button::pro::Font { material::kRoundSmallFont },
            icon_button::pro::FontIcon { "visibility_off" },
            icon_button::pro::ShapeSquare,
            icon_button::pro::ColorStandard,
            icon_button::pro::TypesToggleUnselected,
            icon_button::pro::ToolTip { "切换资产可见性" },
            icon_button::pro::Clickable { [this] {
                if (current_asset_id.empty()) {
                    return;
                }

                toggle_asset_visibility(current_asset_id);
            } },
        };
        sync_visibility_button(false);
        visibility_button->setDisabled(true);

        delete_button = new IconButton {
            icon_button::pro::ThemeManager { manager },
            icon_button::pro::FixedSize { IconButton::kSmallContainerSize },
            icon_button::pro::Font { material::kRoundSmallFont },
            icon_button::pro::FontIcon { "delete" },
            icon_button::pro::ShapeSquare,
            icon_button::pro::ColorStandard,
            icon_button::pro::ToolTip { "删除当前资产" },
            icon_button::pro::Clickable { [this] {
                if (current_asset_id.empty()) {
                    return;
                }

                delete_asset(current_asset_id);
            } },
        };
        delete_button->setDisabled(true);

        return new Row {
            row::pro::Spacing { 10 },
            row::pro::Margin { 5 },
            row::pro::Item { visibility_button.data() },
            row::pro::Item { delete_button.data() },
            row::pro::Stretch { 255 },
        };
    }

    auto make_assets_action(const char* icon, QString const& name, std::function<void()> callback,
        QPointer<IconButton>* button_slot = nullptr, QPointer<Text>* text_slot = nullptr) noexcept
        -> QWidget* {
        auto* button = new IconButton {
            icon_button::pro::ThemeManager { manager },
            icon_button::pro::FixedSize { IconButton::kSmallContainerSize },
            icon_button::pro::FontIcon { icon },
            icon_button::pro::Font { material::round::font, IconButton::kSmallFontIconSize },
            icon_button::pro::Clickable { [callback = std::move(callback)] { callback(); } },
            icon_button::pro::ColorStandard,
            icon_button::pro::ShapeSquare,
        };

        if (button_slot != nullptr) {
            *button_slot = button;
            button->setDisabled(true);
        }

        auto* label = new Text {
            text::pro::ThemeManager { manager },
            text::pro::Text { name },
            text::pro::FixedWidth { 50 },
            text::pro::Alignment { Qt::AlignHCenter },
            text::pro::WordWrap { true },
        };

        if (text_slot != nullptr) {
            *text_slot = label;
        }

        return new Widget {
            widget::pro::Layout<Col> {
                col::pro::Margin { 0 },
                col::pro::Spacing { 5 },
                col::pro::Alignment { Qt::AlignHCenter },
                col::pro::Item { { 0, Qt::AlignHCenter }, button },
                col::pro::Item { { 0, Qt::AlignHCenter }, label },
            },
        };
    }

    auto refresh_assets_list() noexcept -> void {
        auto ids = QStringList { };
        for (const auto& id : assets.get_asset_ids()) {
            ids.append(QString::fromStdString(std::string { id }));
        }
        location_list->setStringList(ids);
    }

    auto last_asset_id() const noexcept -> std::string { return assets.last_asset_id(); }

    auto select_asset(std::string const& id) noexcept -> void { assets_view->select_asset(id); }

    auto sync_visibility_button(bool visible) const noexcept -> void {
        visibility_button->set_selected(!visible);
        visibility_button->set_icon(visible ? "visibility" : "visibility_off");
        visibility_button->setToolTip(visible ? "当前可见" : "当前隐藏");
        visibility_button->update();
        visibility_button->repaint();
    }

    auto sync_asset_buttons() const noexcept -> void {
        const auto enabled = !current_asset_id.empty();

        duplicate_asset_button->setDisabled(!enabled);
        save_asset_button->setDisabled(!enabled);
        save_as_asset_button->setDisabled(!enabled);
    }

    auto sync_visibility_toggle_label() const noexcept -> void {
        if (visibility_toggle_text != nullptr) {
            visibility_toggle_text->setText(assets_visibility ? "隐藏全部" : "显示全部");
        }
    }

    auto toggle_asset_visibility(std::string const& id) noexcept -> void {
        const auto current_visibility = assets.is_asset_visible(id);
        const auto next_visibility    = !current_visibility;
        assets.set_asset_visibility(id, next_visibility);

        if (current_asset_id == id) {
            sync_visibility_button(assets.is_asset_visible(id));
        }
    }

    auto delete_asset(std::string const& id) noexcept -> void {
        const auto name = assets.get_asset_name(id);
        const auto ask  = QMessageBox::question(nullptr, "删除资产",
            QString("确认删除资产 '%1' 吗？该操作不可撤销。").arg(QString::fromStdString(name)),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (ask != QMessageBox::Yes) {
            return;
        }

        if (!assets.remove_asset(id)) {
            QMessageBox::warning(nullptr, "删除失败", "删除当前资产失败。");
            return;
        }

        refresh_assets_list();
        if (current_asset_id == id) {
            clear_asset_detail();
        }
    }

    auto save_asset_as() noexcept -> void {
        if (current_asset_id.empty()) {
            return;
        }

        const auto suggested = assets.get_asset_name(current_asset_id);
        const auto kind      = assets.get_asset_kind(current_asset_id);

        using SaveLocationPicker =
            std::expected<std::string, std::string_view> (*)(std::string const&) noexcept;
        constexpr auto save_location_pickers =
            std::array<std::pair<std::string_view, SaveLocationPicker>, 3> {
                std::pair { pcs::PointsHandle::kKind,
                    &pcs::gui::working::panels::save_pointcloud_location },
                std::pair { pcs::ModelHandle::kKind, &save_model_location },
                std::pair {
                    pcs::PngMapHandle::kKind, &pcs::gui::working::panels::save_png_map_location },
            };

        const auto picker_iter = std::find_if(save_location_pickers.begin(),
            save_location_pickers.end(), [kind](auto const& item) { return item.first == kind; });
        if (picker_iter == save_location_pickers.end()) {
            return;
        }

        auto location = picker_iter->second(suggested);
        if (!location.has_value()) {
            return;
        }

        const auto result = assets.save_asset(current_asset_id, *location);
        if (!result.has_value()) {
            QMessageBox::warning(nullptr, "保存失败", QString::fromStdString(result.error()));
            return;
        }

        refresh_assets_list();
        select_asset(current_asset_id);
    }

    auto save_asset() noexcept -> void {
        if (current_asset_id.empty()) {
            return;
        }

        const auto current_path = assets.get_asset_path(current_asset_id);
        if (current_path.empty() || current_path == "<memory>") {
            save_asset_as();
            return;
        }

        const auto ask_overwrite = QMessageBox::question(nullptr, "确认覆盖",
            QString("将覆盖源文件：\n%1\n是否继续？").arg(QString::fromStdString(current_path)),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (ask_overwrite != QMessageBox::Yes) {
            return;
        }

        auto result = assets.save_asset(current_asset_id, current_path);

        if (!result.has_value()) {
            QMessageBox::warning(nullptr, "保存失败", QString::fromStdString(result.error()));
            return;
        }

        refresh_assets_list();
        select_asset(current_asset_id);
    }

    auto duplicate_asset() noexcept -> void {
        if (current_asset_id.empty()) {
            return;
        }

        const auto kind = assets.get_asset_kind(current_asset_id);

        auto result = assets.clone_asset(current_asset_id);

        if (!result.has_value()) {
            QMessageBox::warning(nullptr, "复制失败", QString::fromStdString(result.error()));
            return;
        }

        const auto post_clone_hooks = std::array {
            std::pair { pcs::PointsHandle::kKind,
                [this](std::string const& asset_id) {
                    if (auto handle = assets.get_handle<pcs::PointsHandle>(asset_id);
                        handle.has_value()) {
                        const auto color = pcs::utility::next_morandi_pointcloud_color();
                        const auto alpha = std::get<3>(handle.value()->get_overall_color());
                        handle.value()->set_overall_color(color.r, color.g, color.b, alpha);
                        assets.update_renderer();
                    }
                } },
        };

        const auto post_clone_iter = std::find_if(post_clone_hooks.begin(), post_clone_hooks.end(),
            [kind](auto const& item) { return item.first == kind; });
        if (post_clone_iter != post_clone_hooks.end()) {
            post_clone_iter->second(*result);
        }

        refresh_assets_list();
        select_asset(*result);
    }

    auto on_asset_selected(std::string const& id) noexcept -> void {
        auto kind = assets.get_asset_kind(id);

        mouse.set_selected_asset(id, kind);

        current_asset_id = id;
        asset_name       = QString::fromStdString(assets.get_asset_name(id));
        asset_path       = QString::fromStdString(assets.get_asset_path(id));

        const auto visible = assets.is_asset_visible(id);

        visibility_button->setDisabled(false);
        sync_visibility_button(visible);
        delete_button->setDisabled(false);

        sync_asset_buttons();
        action_host->bind_asset(kind, id);

        if (auto details = asset_details_registry.provide(assets.get_asset_type(id), assets, id)) {
            asset_type = details->type;
            asset_size = details->size;
            asset_info = details->info;
            return;
        }

        asset_type = "未知";
        asset_size = "未知";
        asset_info = "未知";
    }

    auto clear_asset_detail() noexcept -> void {
        current_asset_id.clear();
        asset_name = "未知";
        asset_type = "未知";
        asset_path = "未知";
        asset_size = "未知";
        asset_info = "未知";

        mouse.clear_selected_asset();

        assets_view->clear_selection();
        sync_visibility_button(false);
        visibility_button->setDisabled(true);
        delete_button->setDisabled(true);

        sync_asset_buttons();
        action_host->clear();
    }

    auto open_location() noexcept -> void {
        const auto previous_last = last_asset_id();

        if (auto result = open_asset_location(open_control.dialog_filter())) {
            if (auto open_result = open_control.open(*result); !open_result.has_value()) {
                QMessageBox::warning(
                    nullptr, "打开失败", QString::fromStdString(open_result.error()));
                return;
            }

            refresh_assets_list();

            const auto current_last = last_asset_id();
            if (!current_last.empty() && current_last != previous_last) {
                select_asset(current_last);
            }
        }
    }

    auto clean_assets() noexcept -> void {
        const auto result =
            QMessageBox::question(nullptr, "确认清空", "确认清空全部资产吗？该操作不可撤销。",
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

        if (result == QMessageBox::Yes) {
            assets.clean_assets();
            refresh_assets_list();
            clear_asset_detail();
        }
    }

    auto hide_assets() noexcept -> void {
        assets_visibility = !assets_visibility;

        for (const auto& id : assets.get_asset_ids()) {
            assets.set_asset_visibility(std::string { id }, assets_visibility);
        }

        sync_visibility_toggle_label();

        if (!current_asset_id.empty()) {
            on_asset_selected(current_asset_id);
        }
    }

    auto reset_view() noexcept -> void { renderer.reset_camera(); }
};

WorkingPanel::WorkingPanel(ThemeManager& manager, pcs::AssetsManager& assets, pcs::Runtime& runtime,
    pcs::Renderer& renderer, pcs::gui::working::OpenControl& open_control,
    pcs::gui::working::AssetDetailsRegistry& asset_details_registry,
    pcs::gui::interaction::Mouse& mouse,
    pcs::gui::working::ActionPanelRegistry& action_panel_registry, MutableDouble& panel_width,
    bool& assets_visibility) noexcept
    : pimpl { std::make_unique<Impl>(*this, manager, assets, runtime, renderer, open_control,
          asset_details_registry, mouse, action_panel_registry, panel_width, assets_visibility) } {
}

WorkingPanel::~WorkingPanel() = default;

auto WorkingPanel::refresh_assets_list() noexcept -> void { pimpl->refresh_assets_list(); }

auto WorkingPanel::select_asset(std::string const& id) noexcept -> void { pimpl->select_asset(id); }

auto WorkingPanel::save_current_asset() noexcept -> void { pimpl->save_asset(); }
