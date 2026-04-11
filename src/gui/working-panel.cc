#include "working-panel.hh"
#include "gui/component/assets-view.hh"

#include <creeper-qt/layout/flow.hh>
#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/utility/material-icon.hh>
#include <creeper-qt/utility/wrapper/mutable-value.hh>
#include <creeper-qt/widget/buttons/icon-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/cards/outlined-card.hh>
#include <creeper-qt/widget/sliders.hh>
#include <creeper-qt/widget/text.hh>
#include <creeper-qt/widget/widget.hh>

#include <expected>

#include <qfiledialog.h>
#include <qmessagebox.h>

#include <spdlog/spdlog.h>

using namespace creeper;

static auto open_pcd_location() noexcept -> std::expected<std::string, std::string_view> {
    const auto pcd_location =
        QFileDialog::getOpenFileName(nullptr, "Open Point Cloud", "", "PCD Diles (*.pcd)");

    if (pcd_location.isEmpty()) {
        return std::unexpected { "User cancelled location selection" };
    }

    return pcd_location.toStdString();
}

auto WorkingPanelComponent(WorkingPanelState& state) noexcept -> QPointer<QWidget> {

    auto& manager = state.manager;
    auto& assets  = state.assets;

    auto current = std::make_shared<pcs::PointsHandle*>();

    auto location_list = new QStringListModel { };

    const auto ThemeManager = theme::pro::ThemeManager { manager };
    const auto font         = QFont { "WenQuanYi Micro Hei Mono", 10 };

    const auto open_location = [&, location_list] {
        if (auto result = open_pcd_location()) {

            assets.open_pointcloud_file(*result);

            auto locations = QStringList { };
            auto generator = assets.get_pointcloud_locations();
            for (auto const& location : generator) {
                locations.append(location.data());
            }
            location_list->setStringList(locations);
        } else {
            spdlog::warn("Failed: {}", result.error());
        }
    };
    const auto clean_pointclouds = [&] {
        const auto result = QMessageBox::question(nullptr, "Confirm Deletion",
            "Are you sure you want to clean all point cloud assets? This action "
            "cannot be undone.",
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

        if (result == QMessageBox::Yes) {
            assets.clean_pointclouds();
            spdlog::info("Clean all pointcloud assets");
        }
    };
    const auto hide_assets = [&] {
        state.pointcloud_visibility = !state.pointcloud_visibility;
        assets.set_pointclouds_visibility(state.pointcloud_visibility);
    };
    const auto reset_view = [&] { state.renderer.reset_camera(); };

    const auto asset_name = std::make_shared<MutableQString>("Unknow");
    const auto asset_path = std::make_shared<MutableQString>("Unknow");
    const auto asset_size = std::make_shared<MutableQString>("Unknow");

    const auto asset_color = std::array {
        std::make_shared<MutableDouble>(),
        std::make_shared<MutableDouble>(),
        std::make_shared<MutableDouble>(),
    };

    const auto asset_selection_callback = [=, &assets](std::string_view view) {
        *asset_path = view.data();
        *asset_name = std::filesystem::path(view.data()).filename().c_str();
        if (auto result = assets.get_pointcloud_handle(view.data())) {
            *current    = result.value();
            *asset_size = QString::number((*current)->get_points_size());
            std::tie(*asset_color[0], *asset_color[1], *asset_color[2]) =
                (*current)->get_overall_color();
        }
    };
    const auto update_asset_color = [=, &assets] {
        if (*current != nullptr) {
            double r = *asset_color[0];
            double g = *asset_color[1];
            double b = *asset_color[2];
            (*current)->set_overall_color(r, g, b);
            assets.update_renderer();
        }
    };

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
                    text::pro::FixedWidth { 40 },
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
                flow::pro::Widget { AssetsAction("delete_sweep", "Clean", clean_pointclouds) },
                flow::pro::Widget { AssetsAction("hide_source", "Hide", hide_assets) },
                flow::pro::Widget { AssetsAction("restart_alt", "Reset View", reset_view) },
            },
        },
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
        return new FilledCard {
            ThemeManager,
            card::pro::LevelLowest,
            card::pro::Layout<Col> {
                col::pro::Margin { 10 },
                col::pro::Item<Text> {
                    ThemeManager,
                    text::pro::Font { font },
                    text::pro::Text { "Asset Detail" },
                    text::pro::Alignment { Qt::AlignHCenter },
                },
                col::pro::SpacingItem { 20 },
                col::pro::Item<FilledCard> {
                    ThemeManager,
                    card::pro::LevelLow,
                    card::pro::Layout<Col> {
                        col::pro::Item { PropRow("Name:", asset_name) },
                        col::pro::Item { PropRow("Size:", asset_size) },
                        col::pro::Item { PropRow("Path:", asset_path) },
                    },
                },
                col::pro::Item<FilledCard> {
                    ThemeManager,
                    card::pro::LevelLow,
                    card::pro::Layout<Col> {
                        col::pro::Item { PropSlider(asset_color[0], "R", update_asset_color) },
                        col::pro::Item { PropSlider(asset_color[1], "G", update_asset_color) },
                        col::pro::Item { PropSlider(asset_color[2], "B", update_asset_color) },
                    },
                },
                col::pro::Stretch { 255 },
            },
        };
    };

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
                    col::pro::Item<AssetsView> {
                        manager,
                        assets,
                        *location_list,
                        asset_selection_callback,
                    },
                    col::pro::Item { { 255 }, AssetDetailView() },
                },
            },
        },
    };
}
