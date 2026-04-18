#include "working-panel.hh"
#include "gui/component/assets-view.hh"
#include "gui/working/action-panels.hh"

#include <creeper-qt/layout/flow.hh>
#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/layout/scroll.hh>
#include <creeper-qt/utility/material-icon.hh>
#include <creeper-qt/utility/wrapper/mutable-value.hh>
#include <creeper-qt/widget/buttons/icon-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/text.hh>
#include <creeper-qt/widget/widget.hh>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <system_error>

#include <QSizePolicy>

#include <QMetaObject>

#include <qfiledialog.h>
#include <qmessagebox.h>

using namespace creeper;

namespace {

constexpr auto kPanelWidthScale = 1.3;
constexpr auto kPanelMinWidth   = 416;
constexpr auto kPanelMaxWidth   = 728;

static auto open_asset_location() noexcept -> std::expected<std::string, std::string_view> {
    const auto location = QFileDialog::getOpenFileName(
        nullptr, "打开资产", "", "资产文件 (*.pcd *.obj);;点云文件 (*.pcd);;模型文件 (*.obj)");

    if (location.isEmpty()) {
        return std::unexpected { "用户取消了文件选择" };
    }

    return location.toStdString();
}

auto asset_kind_text(pcs::AssetKind kind) noexcept -> QString {
    switch (kind) {
    case pcs::AssetKind::Pointcloud:
        return "点云";
    case pcs::AssetKind::Model:
        return "模型";
    case pcs::AssetKind::PngMap:
        return "PNG 地图";
    }

    return "未知";
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

}

auto WorkingPanelComponent(WorkingPanelState& state) noexcept -> QPointer<QWidget> {
    auto& manager = state.manager;
    auto& assets  = state.assets;

    auto current_asset_id = std::make_shared<std::string>();

    auto location_list = new QStringListModel { };

    const auto theme_manager = theme::pro::ThemeManager { manager };
    const auto font          = QFont { "WenQuanYi Micro Hei Mono", 10 };

    auto asset_name = std::make_shared<MutableQString>("未知");
    auto asset_type = std::make_shared<MutableQString>("未知");
    auto asset_path = std::make_shared<MutableQString>("未知");
    auto asset_size = std::make_shared<MutableQString>("未知");
    auto asset_info = std::make_shared<MutableQString>("未知");

    auto visibility_button = std::make_shared<QPointer<IconButton>>();
    auto delete_button     = std::make_shared<QPointer<IconButton>>();

    auto assets_view = std::make_shared<QPointer<AssetsView>>();
    auto action_host = static_cast<pcs::gui::working::ActionPanelHost*>(nullptr);

    const auto refresh_assets_list = [=, &assets] {
        auto ids = QStringList { };
        for (const auto& id : assets.get_asset_ids()) {
            auto key = std::string { id };
            ids.append(QString::fromStdString(key));
        }
        location_list->setStringList(ids);
    };

    const auto last_asset_id = [&assets] {
        auto last = std::string { };
        for (const auto& id : assets.get_asset_ids()) {
            last = std::string { id };
        }
        return last;
    };

    const auto select_asset = [=](std::string const& id) {
        if (assets_view != nullptr && *assets_view != nullptr) {
            (*assets_view)->select_asset(id);
        }
    };

    action_host = new pcs::gui::working::ActionPanelHost(
        pcs::gui::working::ActionPanelContext {
            .manager             = &manager,
            .assets              = &assets,
            .refresh_assets_list = refresh_assets_list,
            .select_asset        = select_asset,
        },
        font);

    const auto prop_row = [&](auto name, auto& prop) {
        return new Row {
            row::pro::Spacing { 8 },
            row::pro::Margin { 5 },
            row::pro::Item<Text> {
                { 0, Qt::AlignTop },
                theme_manager,
                text::pro::Text { name },
                text::pro::Font { font },
                text::pro::Alignment { Qt::AlignTop | Qt::AlignLeft },
                widget::pro::FixedWidth { 54 },
            },
            row::pro::Item<Text> {
                { 1, Qt::AlignTop },
                theme_manager,
                text::pro::Font { font },
                text::pro::WordWrap { true },
                text::pro::Alignment { Qt::AlignTop | Qt::AlignLeft },
                widget::pro::Apply { [](QWidget& self) {
                    self.setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
                } },
                MutableTransform {
                    [](Text& self, const QString& value) {
                        self.setText(fold_text_for_panel(value));
                        self.setToolTip(value);
                    },
                    prop,
                },
            },
        };
    };

    const auto sync_visibility_button = [=](bool visible) {
        if (visibility_button == nullptr || *visibility_button == nullptr) {
            return;
        }

        auto* button = visibility_button->data();
        button->set_types(
            visible ? IconButton::Types::TOGGLE_UNSELECTED : IconButton::Types::TOGGLE_SELECTED);
        button->setDown(false);
        button->set_icon(visible ? "visibility" : "visibility_off");
        button->setToolTip(visible ? "当前可见" : "当前隐藏");
        button->update();
        button->repaint();

        auto guard = QPointer<IconButton> { button };
        QMetaObject::invokeMethod(
            button,
            [guard, visible]() {
                if (guard == nullptr) {
                    return;
                }

                guard->set_types(visible ? IconButton::Types::TOGGLE_UNSELECTED
                                         : IconButton::Types::TOGGLE_SELECTED);
                guard->setDown(false);
                guard->update();
            },
            Qt::QueuedConnection);
    };

    const auto asset_actions_row = [&] {
        *visibility_button = new IconButton {
            icon_button::pro::ThemeManager { manager },
            icon_button::pro::FixedSize { IconButton::kSmallContainerSize },
            icon_button::pro::Font { material::kRoundSmallFont },
            icon_button::pro::FontIcon { "visibility_off" },
            icon_button::pro::ShapeSquare,
            icon_button::pro::ColorStandard,
            icon_button::pro::TypesToggleUnselected,
            icon_button::pro::ToolTip { "切换资产可见性" },
        };
        sync_visibility_button(false);
        (*visibility_button)->setDisabled(true);

        *delete_button = new IconButton {
            icon_button::pro::ThemeManager { manager },
            icon_button::pro::FixedSize { IconButton::kSmallContainerSize },
            icon_button::pro::Font { material::kRoundSmallFont },
            icon_button::pro::FontIcon { "delete" },
            icon_button::pro::ShapeSquare,
            icon_button::pro::ColorStandard,
            icon_button::pro::ToolTip { "删除当前资产" },
        };
        (*delete_button)->setDisabled(true);

        return new Row {
            row::pro::Spacing { 10 },
            row::pro::Margin { 5 },
            row::pro::Item<Text> {
                theme_manager,
                text::pro::Text { "操作:" },
                text::pro::Font { font },
            },
            row::pro::Item { visibility_button->data() },
            row::pro::Item { delete_button->data() },
        };
    };

    const auto asset_detail_view = new FilledCard {
        theme_manager,
        card::pro::LevelLowest,
        card::pro::Layout<Col> {
            col::pro::Margin { 10 },
            col::pro::Spacing { 10 },
            col::pro::Item<Text> {
                theme_manager,
                text::pro::Font { font },
                text::pro::Text { "资产详情" },
                text::pro::Alignment { Qt::AlignHCenter },
            },
            col::pro::Item<FilledCard> {
                theme_manager,
                card::pro::LevelLow,
                card::pro::Layout<Col> {
                    col::pro::Margin { 10 },
                    col::pro::Spacing { 5 },
                    col::pro::Item { prop_row("名称:", asset_name) },
                    col::pro::Item { prop_row("类型:", asset_type) },
                    col::pro::Item { prop_row("大小（MB）:", asset_size) },
                    col::pro::Item { prop_row("信息:", asset_info) },
                    col::pro::Item { prop_row("路径:", asset_path) },
                    col::pro::Item { asset_actions_row() },
                },
            },
            col::pro::Item { action_host->widget() },
            col::pro::Stretch { 255 },
        },
    };

    std::function<void()> clear_asset_detail;

    const auto asset_selection_callback = [=, &assets, &clear_asset_detail](std::string const& id) {
        auto kind = assets.get_asset_kind(id);
        if (!kind.has_value()) {
            clear_asset_detail();
            return;
        }

        *current_asset_id = id;
        *asset_name       = QString::fromStdString(assets.get_asset_name(id).value_or("未知"));
        *asset_type       = asset_kind_text(*kind);
        *asset_path       = QString::fromStdString(assets.get_asset_path(id).value_or("未知"));

        const auto visible = assets.is_asset_visible(id).value_or(true);

        if (visibility_button != nullptr && *visibility_button != nullptr) {
            (*visibility_button)->setDisabled(false);
            sync_visibility_button(visible);
        }

        if (delete_button != nullptr && *delete_button != nullptr) {
            (*delete_button)->setDisabled(false);
        }

        action_host->bind_asset(*kind, id);

        switch (*kind) {
        case pcs::AssetKind::Pointcloud:
            if (auto result = assets.get_pointcloud_handle(id)) {
                auto* handle               = result.value();
                const auto points_count    = static_cast<std::uintmax_t>(handle->get_points_size());
                const auto path            = assets.get_asset_path(id).value_or(std::string { });
                const auto estimated_bytes = points_count * 3U * sizeof(float);

                *asset_size = asset_size_mb_text(path, estimated_bytes);

                const auto is_memory = path == "<memory>";
                *asset_info          = QString("点数: %1 点, 状态: %2")
                                           .arg(static_cast<qulonglong>(points_count))
                                           .arg(is_memory ? "内存中" : "已保存");
            }
            break;
        case pcs::AssetKind::Model:
            if (auto result = assets.get_model_handle(id)) {
                auto* handle        = result.value();
                const auto vertices = static_cast<std::uintmax_t>(handle->get_points_size());
                const auto faces    = static_cast<std::uintmax_t>(handle->get_polys_size());
                const auto path     = assets.get_asset_path(id).value_or(std::string { });
                const auto estimated_bytes =
                    vertices * 3U * sizeof(float) + faces * 3U * sizeof(std::uint32_t);

                *asset_size = asset_size_mb_text(path, estimated_bytes);
                *asset_info = QString("顶点: %1 点, 面数: %2 面")
                                  .arg(static_cast<qulonglong>(vertices))
                                  .arg(static_cast<qulonglong>(faces));
            }
            break;
        case pcs::AssetKind::PngMap:
            if (auto result = assets.get_png_map_handle(id)) {
                auto* handle      = result.value();
                const auto width  = static_cast<std::uintmax_t>(handle->get_width());
                const auto height = static_cast<std::uintmax_t>(handle->get_height());
                const auto path   = assets.get_asset_path(id).value_or(std::string { });

                *asset_size = asset_size_mb_text(path, width * height);
                *asset_info = QString("尺寸: %1 x %2 像素, 分辨率: %3 米/像素, Z: %4")
                                  .arg(static_cast<qulonglong>(width))
                                  .arg(static_cast<qulonglong>(height))
                                  .arg(handle->get_resolution(), 0, 'f', 3)
                                  .arg(handle->get_plane_z(), 0, 'f', 3);
            }
            break;
        }
    };

    *assets_view = new AssetsView {
        manager,
        assets,
        *location_list,
        asset_selection_callback,
    };

    clear_asset_detail = [=] {
        *current_asset_id = { };
        *asset_name       = "未知";
        *asset_type       = "未知";
        *asset_path       = "未知";
        *asset_size       = "未知";
        *asset_info       = "未知";

        if (visibility_button != nullptr && *visibility_button != nullptr) {
            sync_visibility_button(false);
            (*visibility_button)->setDisabled(true);
        }

        if (delete_button != nullptr && *delete_button != nullptr) {
            (*delete_button)->setDisabled(true);
        }

        action_host->clear();
    };

    QObject::connect(visibility_button->data(), &IconButton::clicked, [=, &assets](bool) {
        if (current_asset_id->empty()) {
            return;
        }

        const auto current_visibility = assets.is_asset_visible(*current_asset_id).value_or(true);
        const auto next_visibility    = !current_visibility;
        assets.set_asset_visibility(*current_asset_id, next_visibility);
        sync_visibility_button(assets.is_asset_visible(*current_asset_id).value_or(true));
    });

    QObject::connect(delete_button->data(), &IconButton::clicked, [=, &assets](bool) {
        if (current_asset_id->empty()) {
            return;
        }

        const auto id   = *current_asset_id;
        const auto name = assets.get_asset_name(id).value_or(id);
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
        clear_asset_detail();
    });

    const auto open_location = [=, &assets] {
        const auto previous_last = last_asset_id();

        if (auto result = open_asset_location()) {
            assets.open_file(*result);
            refresh_assets_list();

            const auto current_last = last_asset_id();
            if (!current_last.empty() && current_last != previous_last) {
                select_asset(current_last);
            }
        }
    };

    const auto clean_assets = [=, &assets] {
        const auto result =
            QMessageBox::question(nullptr, "确认清空", "确认清空全部资产吗？该操作不可撤销。",
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

        if (result == QMessageBox::Yes) {
            assets.clean_assets();
            refresh_assets_list();
            clear_asset_detail();
        }
    };

    const auto hide_assets = [=, &assets, &state] {
        state.assets_visibility = !state.assets_visibility;

        for (const auto& id : assets.get_asset_ids()) {
            auto key = std::string { id };
            assets.set_asset_visibility(key, state.assets_visibility);
        }

        if (!current_asset_id->empty()) {
            asset_selection_callback(*current_asset_id);
        }
    };

    const auto reset_view = [&state] { state.renderer.reset_camera(); };

    const auto assets_action = [&](auto icon, auto name, auto&& callback) {
        return new Widget {
            widget::pro::Layout<Col> {
                col::pro::Margin { 0 },
                col::pro::Spacing { 5 },
                col::pro::Item<IconButton> {
                    icon_button::pro::ThemeManager { manager },
                    icon_button::pro::FixedSize { IconButton::kSmallContainerSize },
                    icon_button::pro::FontIcon { icon },
                    icon_button::pro::Font {
                        material::round::font, IconButton::kSmallFontIconSize },
                    icon_button::pro::Clickable { callback },
                    icon_button::pro::ColorStandard,
                    icon_button::pro::ShapeSquare,
                },
                col::pro::Item<Text> {
                    text::pro::ThemeManager { manager },
                    text::pro::Text { name },
                    text::pro::FixedWidth { 50 },
                    text::pro::Alignment { Qt::AlignHCenter },
                    text::pro::WordWrap { true },
                },
            },
        };
    };

    const auto assets_actions_panel = new FilledCard {
        card::pro::ThemeManager { manager },
        card::pro::Radius { 10 },
        card::pro::LevelHigh,
        card::pro::Layout<Col> {
            col::pro::Item<Text> {
                text::pro::ThemeManager { manager },
                text::pro::Font { font },
                text::pro::Alignment { Qt::AlignHCenter },
                text::pro::Text { "资产操作" },
            },
            col::pro::Item<Flow> {
                { 255 },
                flow::pro::RowSpacing { 10 },
                flow::pro::ColSpacing { 10 },
                flow::pro::Alignment { Qt::AlignTop },
                flow::pro::Widget { assets_action("folder_open", "打开", open_location) },
                flow::pro::Widget { assets_action("delete_sweep", "清空", clean_assets) },
                flow::pro::Widget { assets_action("hide_source", "显隐", hide_assets) },
                flow::pro::Widget { assets_action("restart_alt", "重置视角", reset_view) },
            },
        },
    };

    auto scroll_content = new Widget {
        widget::pro::Apply { [](QWidget& self) {
            self.setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        } },
        widget::pro::Layout<Col> {
            col::pro::Margin { 10 },
            col::pro::Spacing { 10 },
            col::pro::Item { assets_actions_panel },
            col::pro::Item { assets_view->data() },
            col::pro::Item { asset_detail_view },
            col::pro::Stretch { 255 },
        },
    };

    auto scrollable = new ScrollArea {
        theme_manager,
        scroll::pro::ScrollBarPolicy { Qt::ScrollBarAsNeeded, Qt::ScrollBarAlwaysOff },
        scroll::pro::Item { scroll_content },
    };

    refresh_assets_list();
    clear_asset_detail();

    return new FilledCard {
        card::pro::ThemeManager { manager },
        widget::pro::MinimumWidth { kPanelMinWidth },
        widget::pro::MaximumWidth { kPanelMaxWidth },
        MutableTransform {
            [](auto& widget, const auto& width) {
                const auto scaled_width =
                    static_cast<int>(std::round(static_cast<double>(width) * kPanelWidthScale));
                const auto clamped_width = std::clamp(scaled_width, kPanelMinWidth, kPanelMaxWidth);
                widget.setFixedWidth(clamped_width);
            },
            state.panel_width,
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
                    col::pro::Item { scrollable },
                },
            },
        },
    };
}
