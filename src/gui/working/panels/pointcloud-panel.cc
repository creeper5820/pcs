#include "gui/working/panels/pointcloud-panel.hh"

#include "core/events/process/pointcloud-to-png-map.hh"
#include "gui/working/panels/common.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/utility/wrapper/mutable-value.hh>
#include <creeper-qt/widget/buttons/outlined-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/sliders.hh>
#include <creeper-qt/widget/text-fields.hh>
#include <creeper-qt/widget/text.hh>

#include <QCoreApplication>
#include <QMetaObject>
#include <QPointer>
#include <QThreadPool>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <tuple>

namespace pcs::gui::working {

namespace {

    constexpr auto kLargePointcloudBytes = std::uintmax_t { 100 } * 1024U * 1024U;

    class PointcloudPanel final : public AssetActionPanel {
    public:
        explicit PointcloudPanel(ActionPanelContext context, QFont const& font)
            : context { std::move(context) }
            , assets { *this->context.assets }
            , mouse { *this->context.mouse } {
            const auto theme = creeper::theme::pro::ThemeManager { *this->context.manager };

            const auto update_color = [this] {
                if (selected_asset_id.empty()) {
                    return;
                }

                if (auto result = assets.get_pointcloud_handle(selected_asset_id)) {
                    auto* handle = result.value();
                    handle->set_overall_color(*color_channels[0], *color_channels[1],
                        *color_channels[2], *color_channels[3]);
                    assets.update_renderer();
                }
            };

            const auto make_slider = [=](std::shared_ptr<creeper::MutableDouble> channel,
                                         std::string_view name) {
                auto measurement          = creeper::Slider::Measurements::Xs();
                measurement.handle_height = 22;

                return new creeper::Row {
                    creeper::row::pro::Spacing { 8 },
                    creeper::row::pro::Item<creeper::Text> {
                        theme,
                        creeper::MutableTransform {
                            [name](creeper::Text& self, double value) {
                                self.setText(QString("%1: %2")
                                        .arg(QString::fromStdString(std::string { name }))
                                        .arg(QString::number(value, 'f', 2)));
                            },
                            channel,
                        },
                        creeper::text::pro::Font { font },
                    },
                    creeper::row::pro::Item<creeper::Slider> {
                        { 255 },
                        theme,
                        creeper::slider::pro::Measurements { measurement },
                        creeper::MutableForward {
                            creeper::slider::pro::Progress { 0 },
                            channel,
                        },
                        creeper::slider::pro::FixedHeight { measurement.minimum_height() },
                        creeper::slider::pro::OnValueChangeFinished {
                            [=](double v) {
                                *channel = v;
                                update_color();
                            },
                        },
                    },
                };
            };

            const auto make_input_row = [=](std::string_view label,
                                            creeper::OutlinedTextField*& input,
                                            QString const& default_value) {
                auto* field = panels::make_parameter_field(theme, font, 116, default_value);
                input       = field;

                return new creeper::Row {
                    creeper::row::pro::Spacing { 8 },
                    creeper::row::pro::Item<creeper::Text> {
                        { 0, Qt::AlignVCenter },
                        theme,
                        creeper::text::pro::Font { font },
                        creeper::text::pro::Text { QString::fromStdString(std::string { label }) },
                        creeper::text::pro::WordWrap { true },
                        creeper::text::pro::Alignment { Qt::AlignVCenter | Qt::AlignLeft },
                        creeper::widget::pro::FixedWidth { 92 },
                    },
                    creeper::row::pro::Item { { 1, Qt::AlignVCenter }, field },
                };
            };

            const auto make_dual_input_row = [=](std::string_view label,
                                                 creeper::OutlinedTextField*& first,
                                                 QString const& first_default,
                                                 creeper::OutlinedTextField*& second,
                                                 QString const& second_default) {
                auto* first_field  = panels::make_parameter_field(theme, font, 56, first_default);
                auto* second_field = panels::make_parameter_field(theme, font, 56, second_default);
                first              = first_field;
                second             = second_field;

                return new creeper::Row {
                    creeper::row::pro::Spacing { 8 },
                    creeper::row::pro::Item<creeper::Text> {
                        { 0, Qt::AlignVCenter },
                        theme,
                        creeper::text::pro::Font { font },
                        creeper::text::pro::Text { QString::fromStdString(std::string { label }) },
                        creeper::text::pro::WordWrap { true },
                        creeper::text::pro::Alignment { Qt::AlignVCenter | Qt::AlignLeft },
                        creeper::widget::pro::FixedWidth { 92 },
                    },
                    creeper::row::pro::Item<creeper::Row> {
                        { 1, Qt::AlignVCenter },
                        creeper::row::pro::Spacing { 4 },
                        creeper::row::pro::Item { first_field },
                        creeper::row::pro::Item { second_field },
                    },
                };
            };

            const auto set_generate_busy = [this](bool busy) {
                generate_button->setDisabled(busy);
                generate_button->setText(busy ? QString::fromUtf8("生成中") : "生成 PNG 地图");
            };

            const auto should_continue_large_generation = [this](std::size_t points_count) -> bool {
                const auto path = assets.get_asset_path(selected_asset_id).value_or("");
                if (!path.empty() && path != "<memory>") {
                    auto error      = std::error_code { };
                    const auto size = std::filesystem::file_size(path, error);
                    if (!error && size > kLargePointcloudBytes) {
                        return panels::confirm_large_pointcloud_warning(size);
                    }
                    return true;
                }

                constexpr auto bytes_per_point = sizeof(double) * 3U;
                const auto estimated_size =
                    static_cast<std::uintmax_t>(points_count) * bytes_per_point;
                if (estimated_size > kLargePointcloudBytes) {
                    return panels::confirm_large_pointcloud_warning(estimated_size);
                }

                return true;
            };

            save_button = new creeper::OutlinedButton {
                theme,
                creeper::widget::pro::FixedHeight { 36 },
                creeper::widget::pro::MinimumWidth { 180 },
                creeper::button::pro::Text { "保存点云" },
            };

            generate_button = new creeper::OutlinedButton {
                theme,
                creeper::widget::pro::FixedHeight { 36 },
                creeper::widget::pro::MinimumWidth { 180 },
                creeper::button::pro::Text { "生成 PNG 地图" },
            };

            QObject::connect(save_button, &creeper::OutlinedButton::clicked, [this](bool) {
                if (selected_asset_id.empty()) {
                    return;
                }

                const auto suggested =
                    assets.get_asset_name(selected_asset_id).value_or("pointcloud.pcd");
                if (auto location = panels::save_pointcloud_location(suggested)) {
                    const auto result = assets.save_pointcloud_asset(selected_asset_id, *location);
                    if (!result.has_value()) {
                        spdlog::error("保存点云资产失败: {}", result.error());
                        return;
                    }

                    this->context.refresh_assets_list();
                    this->context.select_asset(selected_asset_id);
                }
            });

            QObject::connect(generate_button, &creeper::OutlinedButton::clicked,
                [this, set_generate_busy, should_continue_large_generation](bool) {
                    if (selected_asset_id.empty()) {
                        return;
                    }
                    if (!generate_button->isEnabled()) {
                        return;
                    }

                    const auto handle_result = assets.get_pointcloud_handle(selected_asset_id);
                    if (!handle_result.has_value()) {
                        spdlog::error("点云句柄不可用");
                        return;
                    }

                    auto source_points = handle_result.value()->get_positions();
                    if (source_points.empty()) {
                        spdlog::error("点云为空");
                        return;
                    }

                    if (!should_continue_large_generation(source_points.size())) {
                        return;
                    }

                    auto params         = pcs::PngMapParameters { };
                    params.resolution   = panels::parse_double_input(*resolution_input, 0.1, 0.01);
                    params.points_limit = panels::parse_size_input(*points_limit_input, 5, 1);
                    params.height_limit = panels::parse_double_input(*height_limit_input, 0.1, 0.0);
                    params.influence_radius =
                        panels::parse_double_input(*influence_radius_input, 0.08, 0.0);
                    params.z_area_start = panels::parse_double_input(*z_area_start_input, 0.0, 0.0);
                    params.z_area_end   = panels::parse_double_input(*z_area_end_input, 1.0, 0.0);

                    set_generate_busy(true);

                    auto context_copy    = this->context;
                    auto source_asset_id = std::string { selected_asset_id };
                    auto button_guard    = QPointer<creeper::OutlinedButton> { generate_button };
                    auto panel_guard     = QPointer<QWidget> { root };

                    QThreadPool::globalInstance()->start(
                        [context_copy, source_asset_id = std::move(source_asset_id),
                            source_points = std::move(source_points), params, button_guard,
                            panel_guard]() mutable {
                            auto event_context =
                                std::make_unique<pcs::event::ConvertPointcloudToPngMap::Context>();
                            event_context->points     = std::move(source_points);
                            event_context->parameters = params;

                            auto compute_result =
                                std::make_shared<pcs::event::ConvertPointcloudToPngMap::Result>(
                                    pcs::event::ConvertPointcloudToPngMap::runtime_exec(
                                        std::move(event_context)));

                            QMetaObject::invokeMethod(
                                QCoreApplication::instance(),
                                [context_copy, source_asset_id = std::move(source_asset_id),
                                    compute_result, button_guard, panel_guard]() {
                                    if (button_guard) {
                                        button_guard->setDisabled(false);
                                        button_guard->setText("生成 PNG 地图");
                                    }

                                    if (!panel_guard) {
                                        return;
                                    }

                                    if (!compute_result->has_value()) {
                                        spdlog::error(
                                            "点云生成 PNG 地图失败: {}", compute_result->error());
                                        return;
                                    }

                                    const auto create_result =
                                        context_copy.assets->upsert_generated_png_map(
                                            source_asset_id, compute_result->value());
                                    if (!create_result.has_value()) {
                                        spdlog::error(
                                            "创建 PNG 地图资产失败: {}", create_result.error());
                                        return;
                                    }

                                    context_copy.refresh_assets_list();
                                    context_copy.select_asset(*create_result);
                                },
                                Qt::QueuedConnection);
                        });
                });

            root = new creeper::FilledCard {
                theme,
                creeper::card::pro::LevelLow,
                creeper::card::pro::Layout<creeper::Col> {
                    creeper::col::pro::Alignment { Qt::AlignTop },
                    creeper::col::pro::Margin { 10 },
                    creeper::col::pro::Spacing { 10 },
                    creeper::col::pro::Item<creeper::Text> {
                        theme,
                        creeper::text::pro::Font { font },
                        creeper::text::pro::Text { "点云操作" },
                        creeper::text::pro::Alignment { Qt::AlignHCenter },
                    },
                    creeper::col::pro::Item<creeper::FilledCard> {
                        theme,
                        creeper::card::pro::LevelLowest,
                        creeper::card::pro::Layout<creeper::Col> {
                            creeper::col::pro::Margin { 8 },
                            creeper::col::pro::Spacing { 4 },
                            creeper::col::pro::Item { make_slider(color_channels[0], "R") },
                            creeper::col::pro::Item { make_slider(color_channels[1], "G") },
                            creeper::col::pro::Item { make_slider(color_channels[2], "B") },
                            creeper::col::pro::Item { make_slider(color_channels[3], "A") },
                        },
                    },
                    creeper::col::pro::Item<creeper::FilledCard> {
                        theme,
                        creeper::card::pro::LevelLowest,
                        creeper::card::pro::Layout<creeper::Col> {
                            creeper::col::pro::Margin { 8 },
                            creeper::col::pro::Spacing { 4 },
                            creeper::col::pro::Item {
                                make_input_row("分辨率（m）", resolution_input, "0.1") },
                            creeper::col::pro::Item {
                                make_input_row("有效点云数（点）", points_limit_input, "5") },
                            creeper::col::pro::Item {
                                make_input_row("有效高度差（m）", height_limit_input, "0.1") },
                            creeper::col::pro::Item {
                                make_input_row("影响半径（m）", influence_radius_input, "0.08") },
                            creeper::col::pro::Item { make_dual_input_row(
                                "Z 区间（m）", z_area_start_input, "0", z_area_end_input, "1") },
                            creeper::col::pro::Item { generate_button },
                        },
                    },
                    creeper::col::pro::Item { save_button },
                },
            };
        }

        auto widget() const noexcept -> QWidget* override { return root; }

        auto bind_asset(std::string const& id) noexcept -> void override {
            selected_asset_id = id;

            if (auto result = assets.get_pointcloud_handle(selected_asset_id)) {
                auto* handle            = result.value();
                std::tie(*color_channels[0], *color_channels[1], *color_channels[2],
                    *color_channels[3]) = handle->get_overall_color();
            }
        }

        auto clear() noexcept -> void override { selected_asset_id.clear(); }

    private:
        ActionPanelContext context;
        pcs::AssetsManager& assets;
        pcs::gui::interaction::Mouse& mouse;
        std::string selected_asset_id;

        std::array<std::shared_ptr<creeper::MutableDouble>, 4> color_channels {
            std::make_shared<creeper::MutableDouble>(),
            std::make_shared<creeper::MutableDouble>(),
            std::make_shared<creeper::MutableDouble>(),
            std::make_shared<creeper::MutableDouble>(),
        };

        creeper::OutlinedTextField* resolution_input       = nullptr;
        creeper::OutlinedTextField* points_limit_input     = nullptr;
        creeper::OutlinedTextField* height_limit_input     = nullptr;
        creeper::OutlinedTextField* influence_radius_input = nullptr;
        creeper::OutlinedTextField* z_area_start_input     = nullptr;
        creeper::OutlinedTextField* z_area_end_input       = nullptr;

        creeper::OutlinedButton* save_button     = nullptr;
        creeper::OutlinedButton* generate_button = nullptr;
        creeper::FilledCard* root                = nullptr;
    };

}

auto make_pointcloud_panel(ActionPanelContext context, QFont const& font)
    -> std::unique_ptr<AssetActionPanel> {
    return std::make_unique<PointcloudPanel>(std::move(context), font);
}

}
