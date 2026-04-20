#include "gui/working/panels/model-panel.hh"

#include "core/events/process/model-to-pointcloud.hh"
#include "gui/working/panels/common.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/widget/buttons/outlined-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/text.hh>

#include <QTimer>
#include <spdlog/spdlog.h>

#include <chrono>
#include <future>

using namespace creeper;
namespace pcs::gui::working {

namespace {

    class ModelPanelBody final : public AssetActionPanel {
    public:
        explicit ModelPanelBody(ActionPanelContext context, QFont const& font)
            : context { std::move(context) }
            , assets { *this->context.assets }
            , runtime { *this->context.runtime }
            , mouse { *this->context.mouse } {
            namespace ob = creeper::outlined_button::pro;
            namespace bp = creeper::button::pro;
            const auto theme = ob::ThemeManager { *this->context.manager };

            density_row = new panels::CompactFieldRow(theme, font, "点云密度（倍）", 92, 116, "10");
            density_input = &density_row->field();
            sample_distance_row =
                new panels::CompactFieldRow(theme, font, "采样间距（m）", 92, 116, "0");
            sample_distance_input = &sample_distance_row->field();
            unit_scale_row =
                new panels::CompactFieldRow(theme, font, "缩放比例（倍）", 92, 116, "1");
            unit_scale_input = &unit_scale_row->field();
            max_points_row =
                new panels::CompactFieldRow(theme, font, "最大点数（点）", 92, 116, "0");
            max_points_input = &max_points_row->field();

            convert_button = new creeper::OutlinedButton {
                theme,
                ob::FixedHeight { 36 },
                ob::MinimumWidth { 180 },
                ob::Text { "转换为点云" },
                bp::Clickable { [this] {
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

                    auto event       = pcs::event::ConvertModelToPointcloud { };
                    event.model      = handle.value()->model_data();
                    event.parameters = parameters;

                    const auto source_id = selected_asset_id;
                    auto future =
                        std::make_shared<std::future<pcs::event::ConvertModelToPointcloud::Result>>(
                            runtime.submit(std::move(event)));

                    set_convert_busy(true);

                    auto* watcher = new QTimer { convert_button };
                    watcher->setInterval(30);
                    QObject::connect(
                        watcher, &QTimer::timeout, [this, watcher, future, source_id]() mutable {
                            if (future->wait_for(std::chrono::seconds { 0 })
                                != std::future_status::ready) {
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

                            auto upsert_result =
                                assets.upsert_generated_pointcloud(source_id, result.value());
                            if (!upsert_result.has_value()) {
                                spdlog::error("模型转点云失败: {}", upsert_result.error());
                                return;
                            }

                            this->context.refresh_assets_list();
                            this->context.select_asset(*upsert_result);
                        });
                    watcher->start();
                } },
            };

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
                            creeper::col::pro::Item { density_row },
                            creeper::col::pro::Item { sample_distance_row },
                            creeper::col::pro::Item { unit_scale_row },
                            creeper::col::pro::Item { max_points_row },
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

        panels::CompactFieldRow* density_row         = nullptr;
        panels::CompactFieldRow* sample_distance_row = nullptr;
        panels::CompactFieldRow* unit_scale_row      = nullptr;
        panels::CompactFieldRow* max_points_row      = nullptr;

        creeper::OutlinedTextField* density_input         = nullptr;
        creeper::OutlinedTextField* sample_distance_input = nullptr;
        creeper::OutlinedTextField* unit_scale_input      = nullptr;
        creeper::OutlinedTextField* max_points_input      = nullptr;

        creeper::OutlinedButton* convert_button = nullptr;
        creeper::FilledCard* root               = nullptr;
    };

}

struct ModelPanel::Impl {
    std::unique_ptr<AssetActionPanel> panel;
};

ModelPanel::~ModelPanel() noexcept = default;

auto ModelPanel::widget() const noexcept -> QWidget* { return pimpl->panel->widget(); }

auto ModelPanel::bind_asset(std::string const& id) noexcept -> void {
    pimpl->panel->bind_asset(id);
}

auto ModelPanel::clear() noexcept -> void { pimpl->panel->clear(); }

ModelPanel::ModelPanel(ActionPanelContext context, QFont const& font) noexcept
    : pimpl { std::make_unique<Impl>() } {
    pimpl->panel = std::make_unique<ModelPanelBody>(std::move(context), font);
}

}
