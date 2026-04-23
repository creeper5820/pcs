#include "gui/working/panels/model-panel.hh"

#include "core/events/process/model-to-pointcloud.hh"
#include "core/handle/model.hh"
#include "core/handle/points.hh"
#include "gui/working/panels/common.hh"
#include "utility/morandi-pointcloud-color.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/widget/buttons/outlined-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/text.hh>

#include <QTimer>
#include <spdlog/spdlog.h>

#include <chrono>
#include <filesystem>
#include <future>

using namespace creeper;
namespace pcs::gui::working {

namespace {

    class ModelPanelBody final : public AssetActionPanel {
    public:
        explicit ModelPanelBody(ActionPanelContext context, QFont const& font)
            : context { std::move(context) }
            , assets { *this->context.assets }
            , runtime { *this->context.runtime } {
            using namespace creeper;
            namespace ob     = outlined_button::pro;
            namespace bp     = button::pro;
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

            convert_button = new OutlinedButton {
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
                    parameters.unit_scale =
                        panels::parse_double_input(*unit_scale_input, 1.0, 1e-6);
                    parameters.max_points = panels::parse_size_input(*max_points_input, 0, 0);

                    if (selected_handle == nullptr) {
                        spdlog::error("模型转点云失败: 模型资产不可用");
                        return;
                    }

                    auto event       = pcs::event::ConvertModelToPointcloud { };
                    event.model      = selected_handle->model_data();
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

                            auto pointcloud  = std::make_unique<PointsHandle>();
                            auto load_result = pointcloud->load_from_positions(result.value());
                            if (!load_result.has_value()) {
                                spdlog::error("模型转点云失败: {}", load_result.error());
                                return;
                            }

                            const auto color = utility::next_morandi_pointcloud_color();
                            pointcloud->set_overall_color(color.r, color.g, color.b);

                            auto source_name = assets.get_asset_name(source_id);
                            auto derived     = std::filesystem::path(source_name);
                            if (derived.empty()) {
                                derived = "converted-pointcloud.pcd";
                            } else {
                                derived.replace_extension(".pcd");
                            }

                            auto generated_id = assets.register_asset<PointsHandle>(
                                std::move(pointcloud), derived.filename().string(), { }, false);

                            this->context.refresh_assets_list();
                            this->context.select_asset(generated_id);
                        });
                    watcher->start();
                } },
            };

            root = new FilledCard {
                theme,
                card::pro::LevelLow,
                card::pro::Layout<Col> {
                    col::pro::Alignment { Qt::AlignTop },
                    col::pro::Margin { 10 },
                    col::pro::Spacing { 10 },
                    col::pro::Item<Text> {
                        theme,
                        text::pro::Font { font },
                        text::pro::Text { "模型操作" },
                        text::pro::Alignment { Qt::AlignHCenter },
                    },
                    col::pro::Item<FilledCard> {
                        theme,
                        card::pro::LevelLowest,
                        card::pro::Layout<Col> {
                            col::pro::Margin { 24 },
                            col::pro::Spacing { 4 },
                            col::pro::Item { density_row },
                            col::pro::Item { sample_distance_row },
                            col::pro::Item { unit_scale_row },
                            col::pro::Item { max_points_row },
                        },
                    },
                    col::pro::Item { convert_button },
                },
            };
        }

        auto widget() const noexcept -> QWidget* override { return root; }

        auto bind_asset(std::string const& id) noexcept -> void override {
            selected_asset_id = id;
            auto handle       = assets.get_handle<ModelHandle>(id);
            selected_handle   = handle.has_value() ? handle.value() : nullptr;
        }

        auto clear() noexcept -> void override {
            selected_asset_id.clear();
            selected_handle = nullptr;
        }

    private:
        auto set_convert_busy(bool busy) noexcept -> void {
            convert_button->setDisabled(busy);
            convert_button->setText(busy ? QString::fromUtf8("生成中...") : "转换为点云");
        }

        ActionPanelContext context;
        pcs::AssetsManager& assets;
        pcs::Runtime& runtime;
        std::string selected_asset_id;
        ModelHandle* selected_handle = nullptr;

        panels::CompactFieldRow* density_row         = nullptr;
        panels::CompactFieldRow* sample_distance_row = nullptr;
        panels::CompactFieldRow* unit_scale_row      = nullptr;
        panels::CompactFieldRow* max_points_row      = nullptr;

        OutlinedTextField* density_input         = nullptr;
        OutlinedTextField* sample_distance_input = nullptr;
        OutlinedTextField* unit_scale_input      = nullptr;
        OutlinedTextField* max_points_input      = nullptr;

        OutlinedButton* convert_button = nullptr;
        FilledCard* root               = nullptr;
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
