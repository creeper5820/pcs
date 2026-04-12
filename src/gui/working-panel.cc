#include "working-panel.hh"
#include "gui/component/assets-view.hh"

#include <creeper-qt/layout/flow.hh>
#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/utility/material-icon.hh>
#include <creeper-qt/utility/wrapper/mutable-value.hh>
#include <creeper-qt/widget/buttons/icon-button.hh>
#include <creeper-qt/widget/buttons/outlined-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/cards/outlined-card.hh>
#include <creeper-qt/widget/sliders.hh>
#include <creeper-qt/widget/switch.hh>
#include <creeper-qt/widget/text.hh>
#include <creeper-qt/widget/widget.hh>

#include <array>
#include <expected>
#include <filesystem>
#include <tuple>

#include <qfiledialog.h>
#include <qmessagebox.h>

#include <spdlog/spdlog.h>

using namespace creeper;

static auto open_asset_location() noexcept -> std::expected<std::string, std::string_view> {
    const auto location = QFileDialog::getOpenFileName(nullptr, "Open Asset", "",
        "Asset Files (*.pcd *.obj);;Point Cloud Files (*.pcd);;Model Files (*.obj)");

    if (location.isEmpty()) {
        return std::unexpected { "User cancelled location selection" };
    }

    return location.toStdString();
}

static auto save_pointcloud_location(std::string const& suggested_name) noexcept
    -> std::expected<std::string, std::string_view> {
    auto filename = std::filesystem::path(suggested_name);
    if (filename.extension() != ".pcd") {
        filename.replace_extension(".pcd");
    }

    const auto location = QFileDialog::getSaveFileName(nullptr, "Save Point Cloud",
        QString::fromStdString(filename.string()), "PCD Files (*.pcd)");

    if (location.isEmpty()) {
        return std::unexpected { "User cancelled pointcloud save" };
    }

    return location.toStdString();
}

auto WorkingPanelComponent(WorkingPanelState& state) noexcept -> QPointer<QWidget> {
    auto& manager = state.manager;
    auto& assets  = state.assets;

    auto current_asset_id = std::make_shared<std::string>();

    auto location_list = new QStringListModel { };

    const auto ThemeManager = theme::pro::ThemeManager { manager };
    const auto font         = QFont { "WenQuanYi Micro Hei Mono", 10 };

    auto asset_name = std::make_shared<MutableQString>("Unknown");
    auto asset_type = std::make_shared<MutableQString>("Unknown");
    auto asset_path = std::make_shared<MutableQString>("Unknown");
    auto asset_size = std::make_shared<MutableQString>("Unknown");
    auto asset_info = std::make_shared<MutableQString>("Unknown");

    const auto asset_color = std::array {
        std::make_shared<MutableDouble>(),
        std::make_shared<MutableDouble>(),
        std::make_shared<MutableDouble>(),
    };

    auto visibility_switch       = static_cast<Switch*>(nullptr);
    auto save_button             = static_cast<OutlinedButton*>(nullptr);
    auto convert_button          = static_cast<OutlinedButton*>(nullptr);
    auto pointcloud_actions_card = static_cast<FilledCard*>(nullptr);
    auto pointcloud_color_card   = static_cast<FilledCard*>(nullptr);
    auto other_actions_card      = static_cast<FilledCard*>(nullptr);
    auto assets_view             = static_cast<AssetsView*>(nullptr);

    const auto refresh_assets_list = [=, &assets] {
        auto ids = QStringList { };
        for (const auto& id : assets.get_asset_ids()) {
            ids.append(QString::fromStdString(std::string { id }));
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

    const auto update_asset_color = [=, &assets] {
        if (current_asset_id->empty()) {
            return;
        }

        if (auto result = assets.get_pointcloud_handle(*current_asset_id)) {
            auto* handle = result.value();
            handle->set_overall_color(*asset_color[0], *asset_color[1], *asset_color[2]);
            assets.update_renderer();
        }
    };

    const auto AssetDetailView = [&] {
        const auto PropRow = [&](auto name, auto& prop) {
            return new Row {
                row::pro::Spacing { 5 },
                row::pro::Margin { 5 },
                row::pro::Item<Text> {
                    ThemeManager,
                    text::pro::Text { name },
                    text::pro::Font { font },
                },
                row::pro::Item<Text> {
                    ThemeManager,
                    text::pro::Font { font },
                    MutableForward {
                        text::pro::Text { },
                        prop,
                    },
                },
            };
        };

        const auto VisibleRow = [&] {
            visibility_switch = new Switch {
                ThemeManager,
                widget::pro::FixedSize { QSize { 52, 32 } },
                _switch::pro::Checked { false },
                _switch::pro::Disabled { true },
            };

            return new Row {
                row::pro::Spacing { 10 },
                row::pro::Margin { 5 },
                row::pro::Item<Text> {
                    ThemeManager,
                    text::pro::Text { "Visible:" },
                    text::pro::Font { font },
                },
                row::pro::Item { visibility_switch },
            };
        };

        const auto PropSlider = [&](std::shared_ptr<MutableDouble> channel, auto name, auto f) {
            auto measurement          = Slider::Measurements::Xs();
            measurement.handle_height = 22;

            return new Row {
                row::pro::Spacing { 10 },
                row::pro::Item<Text> {
                    ThemeManager,
                    MutableTransform {
                        [name](Text& self, double v) {
                            auto text = QString("%1: %2").arg(name).arg(QString::number(v, 'f', 2));
                            self.setText(text);
                        },
                        channel,
                    },
                    text::pro::Font { font },
                },
                row::pro::Item<Slider> {
                    { 255 },
                    ThemeManager,
                    slider::pro::Measurements { measurement },
                    MutableForward {
                        slider::pro::Progress { 0 },
                        channel,
                    },
                    slider::pro::FixedHeight { measurement.minimum_height() },
                    slider::pro::OnValueChangeFinished {
                        [=](double v) { *channel = v, f(); },
                    },
                },
            };
        };

        pointcloud_color_card = new FilledCard {
            ThemeManager,
            card::pro::LevelLow,
            card::pro::Layout<Col> {
                col::pro::Margin { 10 },
                col::pro::Spacing { 5 },
                col::pro::Item<Text> {
                    ThemeManager,
                    text::pro::Font { font },
                    text::pro::Text { "Pointcloud Appearance" },
                    text::pro::Alignment { Qt::AlignHCenter },
                },
                col::pro::Item { PropSlider(asset_color[0], "R", update_asset_color) },
                col::pro::Item { PropSlider(asset_color[1], "G", update_asset_color) },
                col::pro::Item { PropSlider(asset_color[2], "B", update_asset_color) },
            },
        };
        pointcloud_color_card->setVisible(false);

        save_button = new OutlinedButton {
            ThemeManager,
            widget::pro::FixedHeight { 36 },
            widget::pro::MinimumWidth { 180 },
            button::pro::Text { "Save Pointcloud" },
        };

        pointcloud_actions_card = new FilledCard {
            ThemeManager,
            card::pro::LevelLow,
            card::pro::Layout<Col> {
                col::pro::Margin { 10 },
                col::pro::Spacing { 5 },
                col::pro::Item<Text> {
                    ThemeManager,
                    text::pro::Font { font },
                    text::pro::Text { "Pointcloud Actions" },
                    text::pro::Alignment { Qt::AlignHCenter },
                },
                col::pro::Item { save_button },
            },
        };
        pointcloud_actions_card->setVisible(false);

        convert_button = new OutlinedButton {
            ThemeManager,
            widget::pro::FixedHeight { 36 },
            widget::pro::MinimumWidth { 180 },
            button::pro::Text { "Convert To Pointcloud" },
        };

        other_actions_card = new FilledCard {
            ThemeManager,
            card::pro::LevelLow,
            card::pro::Layout<Col> {
                col::pro::Margin { 10 },
                col::pro::Spacing { 5 },
                col::pro::Item<Text> {
                    ThemeManager,
                    text::pro::Font { font },
                    text::pro::Text { "Other Actions" },
                    text::pro::Alignment { Qt::AlignHCenter },
                },
                col::pro::Item { convert_button },
            },
        };
        other_actions_card->setVisible(false);

        return new FilledCard {
            ThemeManager,
            card::pro::LevelLowest,
            card::pro::Layout<Col> {
                col::pro::Margin { 10 },
                col::pro::Spacing { 10 },
                col::pro::Item<Text> {
                    ThemeManager,
                    text::pro::Font { font },
                    text::pro::Text { "Asset Detail" },
                    text::pro::Alignment { Qt::AlignHCenter },
                },
                col::pro::Item<FilledCard> {
                    ThemeManager,
                    card::pro::LevelLow,
                    card::pro::Layout<Col> {
                        col::pro::Margin { 10 },
                        col::pro::Spacing { 5 },
                        col::pro::Item { PropRow("Name:", asset_name) },
                        col::pro::Item { PropRow("Type:", asset_type) },
                        col::pro::Item { PropRow("Size:", asset_size) },
                        col::pro::Item { PropRow("Info:", asset_info) },
                        col::pro::Item { PropRow("Path:", asset_path) },
                        col::pro::Item { VisibleRow() },
                    },
                },
                col::pro::Item { pointcloud_color_card },
                col::pro::Item { pointcloud_actions_card },
                col::pro::Item { other_actions_card },
                col::pro::Stretch { 255 },
            },
        };
    };

    const auto asset_detail_view = AssetDetailView();

    const auto clear_asset_detail = [=] {
        *current_asset_id = { };
        *asset_name       = "Unknown";
        *asset_type       = "Unknown";
        *asset_path       = "Unknown";
        *asset_size       = "Unknown";
        *asset_info       = "Unknown";

        if (visibility_switch != nullptr) {
            visibility_switch->set_checked(false);
            visibility_switch->set_disabled(true);
        }

        if (pointcloud_actions_card != nullptr) {
            pointcloud_actions_card->setVisible(false);
        }
        if (pointcloud_color_card != nullptr) {
            pointcloud_color_card->setVisible(false);
        }
        if (other_actions_card != nullptr) {
            other_actions_card->setVisible(false);
        }
    };

    const auto asset_selection_callback = [=, &assets](std::string const& id) {
        auto kind = assets.get_asset_kind(id);
        if (!kind.has_value()) {
            clear_asset_detail();
            return;
        }

        *current_asset_id = id;
        *asset_name       = QString::fromStdString(assets.get_asset_name(id).value_or("Unknown"));
        *asset_type       = *kind == pcs::AssetKind::Pointcloud ? "PointCloud" : "Model";
        *asset_path       = QString::fromStdString(assets.get_asset_path(id).value_or("Unknown"));

        if (visibility_switch != nullptr) {
            visibility_switch->set_disabled(false);
            visibility_switch->set_checked(assets.is_asset_visible(id).value_or(true));
        }

        if (pointcloud_actions_card != nullptr) {
            pointcloud_actions_card->setVisible(false);
        }
        if (pointcloud_color_card != nullptr) {
            pointcloud_color_card->setVisible(false);
        }
        if (other_actions_card != nullptr) {
            other_actions_card->setVisible(false);
        }

        if (*kind == pcs::AssetKind::Pointcloud) {
            if (auto result = assets.get_pointcloud_handle(id)) {
                auto* handle = result.value();

                *asset_size = QString::number(handle->get_points_size());
                *asset_info = assets.get_asset_path(id).value_or("Unknown") == "<memory>" ? "State:"
                                                                                            " Memor"
                                                                                            "y"
                                                                                          : "State:"
                                                                                            " Save"
                                                                                            "d";

                std::tie(*asset_color[0], *asset_color[1], *asset_color[2]) =
                    handle->get_overall_color();
            }

            if (pointcloud_actions_card != nullptr) {
                pointcloud_actions_card->setVisible(true);
            }
            if (pointcloud_color_card != nullptr) {
                pointcloud_color_card->setVisible(true);
            }
            return;
        }

        if (auto result = assets.get_model_handle(id)) {
            auto* handle = result.value();
            *asset_size  = QString::number(handle->get_points_size());
            *asset_info  = QString("Faces: %1").arg(handle->get_polys_size());
        }

        if (other_actions_card != nullptr) {
            other_actions_card->setVisible(true);
        }
    };

    QObject::connect(visibility_switch, &QAbstractButton::clicked, [=, &assets](bool) {
        if (!current_asset_id->empty()) {
            assets.set_asset_visibility(*current_asset_id, visibility_switch->checked());
        }
    });

    assets_view = new AssetsView {
        manager,
        assets,
        *location_list,
        asset_selection_callback,
    };

    const auto save_pointcloud = [=, &assets] {
        if (current_asset_id->empty()) {
            return;
        }

        auto suggested_name = assets.get_asset_name(*current_asset_id).value_or("pointcloud.pcd");
        if (auto result = save_pointcloud_location(suggested_name)) {
            auto save_result = assets.save_pointcloud_asset(*current_asset_id, *result);
            if (!save_result.has_value()) {
                spdlog::error("Failed to save pointcloud asset: {}", save_result.error());
                return;
            }

            refresh_assets_list();
            if (assets_view != nullptr) {
                assets_view->select_asset(*current_asset_id);
            }
        } else {
            spdlog::warn("Failed: {}", result.error());
        }
    };

    const auto convert_model = [=, &assets] {
        if (current_asset_id->empty()) {
            return;
        }

        auto result = assets.convert_model_to_pointcloud(*current_asset_id);
        if (!result.has_value()) {
            spdlog::error("Failed to convert model asset: {}", result.error());
            return;
        }

        refresh_assets_list();
        if (assets_view != nullptr) {
            assets_view->select_asset(*result);
        }
    };

    QObject::connect(save_button, &QAbstractButton::clicked, [=](bool) { save_pointcloud(); });
    QObject::connect(convert_button, &QAbstractButton::clicked, [=](bool) { convert_model(); });

    const auto open_location = [=, &assets] {
        const auto previous_last = last_asset_id();

        if (auto result = open_asset_location()) {
            assets.open_file(*result);
            refresh_assets_list();

            const auto current_last = last_asset_id();
            if (!current_last.empty() && current_last != previous_last && assets_view != nullptr) {
                assets_view->select_asset(current_last);
            }
        } else {
            spdlog::warn("Failed: {}", result.error());
        }
    };

    const auto clean_assets = [=, &assets] {
        const auto result = QMessageBox::question(nullptr, "Confirm Deletion",
            "Are you sure you want to clean all assets? This action cannot be undone.",
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

        if (result == QMessageBox::Yes) {
            assets.clean_assets();
            refresh_assets_list();
            clear_asset_detail();
            spdlog::info("Clean all assets");
        }
    };

    const auto hide_assets = [=, &assets, &state] {
        state.assets_visibility = !state.assets_visibility;

        for (const auto& id : assets.get_asset_ids()) {
            assets.set_asset_visibility(std::string { id }, state.assets_visibility);
        }

        if (!current_asset_id->empty()) {
            asset_selection_callback(*current_asset_id);
        }
    };

    const auto reset_view = [&state] { state.renderer.reset_camera(); };

    const auto AssetsAction = [&](auto icon, auto name, auto&& callback) {
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

    const auto AssetsActionsPanel = new FilledCard {
        card::pro::ThemeManager { manager },
        card::pro::Radius { 10 },
        card::pro::LevelHigh,
        card::pro::Layout<Col> {
            col::pro::Item<Text> {
                text::pro::ThemeManager { manager },
                text::pro::Font { font },
                text::pro::Alignment { Qt::AlignHCenter },
                text::pro::Text { "Assets Actions" },
            },
            col::pro::Item<Flow> {
                { 255 },
                flow::pro::RowSpacing { 10 },
                flow::pro::ColSpacing { 10 },
                flow::pro::Alignment { Qt::AlignTop },
                flow::pro::Widget { AssetsAction("folder_open", "Open", open_location) },
                flow::pro::Widget { AssetsAction("delete_sweep", "Clean", clean_assets) },
                flow::pro::Widget { AssetsAction("hide_source", "Hide", hide_assets) },
                flow::pro::Widget { AssetsAction("restart_alt", "Reset View", reset_view) },
            },
        },
    };

    refresh_assets_list();
    clear_asset_detail();

    return new FilledCard {
        card::pro::ThemeManager { manager },
        MutableTransform {
            [](auto& widget, const auto& width) { widget.setFixedWidth(width); },
            state.panel_width,
        },
        card::pro::Radius { 0 },
        card::pro::Layout<Col> {
            col::pro::Margin { 0 },
            col::pro::Item<FilledCard> {
                card::pro::ThemeManager { manager },
                card::pro::Radius { 10 },
                card::pro::Layout<Col> {
                    col::pro::Margin { 10 },
                    col::pro::Spacing { 10 },
                    col::pro::Item { AssetsActionsPanel },
                    col::pro::Item { assets_view },
                    col::pro::Item { { 255 }, asset_detail_view },
                },
            },
        },
    };
}
