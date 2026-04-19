#include "gui/working/panels/model-panel.hh"

#include "core/events/process/model-to-pointcloud.hh"
#include "gui/working/panels/common.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/widget/buttons/outlined-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/text.hh>

#include <spdlog/spdlog.h>
#include <QTimer>

#include <chrono>
#include <future>

namespace pcs::gui::working {

namespace {

    class ModelPanel final : public AssetActionPanel {
    public:
        explicit ModelPanel(ActionPanelContext context, QFont const& font)
            : context { std::move(context) }
            , assets { *this->context.assets }
            , runtime { *this->context.runtime }
            , mouse { *this->context.mouse } {
            const auto theme = creeper::theme::pro::ThemeManager { *this->context.manager };

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

            convert_button = new creeper::OutlinedButton {
                theme,
                creeper::widget::pro::FixedHeight { 36 },
                creeper::widget::pro::MinimumWidth { 180 },
                creeper::button::pro::Text { "转换为点云" },
            };

            QObject::connect(convert_button, &creeper::OutlinedButton::clicked, [this](bool) {
                if (selected_asset_id.empty()) {
                    return;
                }
                if (!convert_button->isEnabled()) {
                    return;
                }

                auto parameters    = pcs::ModelToPointcloudParameters { };
                parameters.density = panels::parse_double_input(*density_input, 10.0, 0.01);
                parameters.sample_distance =
                    panels::parse_double_input(*sample_distance_input, 0.0, 0.0);
                parameters.unit_scale = panels::parse_double_input(*unit_scale_input, 1.0, 1e-6);
                parameters.max_points = panels::parse_size_input(*max_points_input, 0, 0);

                const auto handle = assets.get_model_handle(selected_asset_id);
                if (!handle.has_value() || handle.value() == nullptr) {
                    spdlog::error("模型转点云失败: 模型资产不可用");
                    return;
                }

                auto event_context        = std::make_unique<pcs::event::ConvertModelToPointcloud::Context>();
                event_context->model      = handle.value()->model_data();
                event_context->parameters = parameters;

                const auto source_id = selected_asset_id;
                auto future          = std::make_shared<std::future<pcs::event::ConvertModelToPointcloud::Result>>(
                    runtime.submit<pcs::event::ConvertModelToPointcloud>(std::move(event_context)));

                set_convert_busy(true);

                auto* watcher = new QTimer { convert_button };
                watcher->setInterval(30);
                QObject::connect(watcher, &QTimer::timeout,
                    [this, watcher, future, source_id]() mutable {
                        if (future->wait_for(std::chrono::seconds { 0 }) != std::future_status::ready) {
                            return;
                        }

                        watcher->stop();
                        watcher->deleteLater();
                        set_convert_busy(false);

                        auto result = future->get();
                        if (!result.has_value()) {
                            spdlog::error("模型转点云失败: {}", result.error());
                            return;
                        }

                        auto upsert_result = assets.upsert_generated_pointcloud(source_id, result.value());
                        if (!upsert_result.has_value()) {
                            spdlog::error("模型转点云失败: {}", upsert_result.error());
                            return;
                        }

                        this->context.refresh_assets_list();
                        this->context.select_asset(*upsert_result);
                    });
                watcher->start();
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
                        creeper::text::pro::Text { "模型操作" },
                        creeper::text::pro::Alignment { Qt::AlignHCenter },
                    },
                    creeper::col::pro::Item<creeper::FilledCard> {
                        theme,
                        creeper::card::pro::LevelLowest,
                        creeper::card::pro::Layout<creeper::Col> {
                            creeper::col::pro::Margin { 8 },
                            creeper::col::pro::Spacing { 4 },
                            creeper::col::pro::Item {
                                make_input_row("点云密度（倍）", density_input, "10") },
                            creeper::col::pro::Item {
                                make_input_row("采样间距（m）", sample_distance_input, "0") },
                            creeper::col::pro::Item {
                                make_input_row("缩放比例（倍）", unit_scale_input, "1") },
                            creeper::col::pro::Item {
                                make_input_row("最大点数（点）", max_points_input, "0") },
                        },
                    },
                    creeper::col::pro::Item { convert_button },
                },
            };
        }

        auto widget() const noexcept -> QWidget* override { return root; }

        auto bind_asset(std::string const& id) noexcept -> void override { selected_asset_id = id; }

        auto clear() noexcept -> void override { selected_asset_id.clear(); }

    private:
        auto set_convert_busy(bool busy) noexcept -> void {
            convert_button->setDisabled(busy);
            convert_button->setText(busy ? QString::fromUtf8("生成中...") : "转换为点云");
        }

        ActionPanelContext context;
        pcs::AssetsManager& assets;
        pcs::Runtime& runtime;
        pcs::gui::interaction::Mouse& mouse;
        std::string selected_asset_id;

        creeper::OutlinedTextField* density_input         = nullptr;
        creeper::OutlinedTextField* sample_distance_input = nullptr;
        creeper::OutlinedTextField* unit_scale_input      = nullptr;
        creeper::OutlinedTextField* max_points_input      = nullptr;

        creeper::OutlinedButton* convert_button = nullptr;
        creeper::FilledCard* root               = nullptr;
    };

}

auto make_model_panel(ActionPanelContext context, QFont const& font)
    -> std::unique_ptr<AssetActionPanel> {
    return std::make_unique<ModelPanel>(std::move(context), font);
}

}
