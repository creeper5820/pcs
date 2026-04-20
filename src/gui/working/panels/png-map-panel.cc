#include "gui/working/panels/png-map-panel.hh"

#include "gui/interaction/png-edit-tools.hh"
#include "gui/working/panels/common.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/layout/stacked.hh>
#include <creeper-qt/utility/material-icon.hh>
#include <creeper-qt/widget/buttons/icon-button.hh>
#include <creeper-qt/widget/buttons/outlined-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/dropdown-menu.hh>
#include <creeper-qt/widget/text.hh>
#include <creeper-qt/widget/widget.hh>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <vector>

namespace pcs::gui::working {

namespace {

    auto origin_summary(pcs::PngMapFrameConfig const& config) noexcept -> QString {
        if (config.origin_pixel_x.has_value() && config.origin_pixel_y.has_value()) {
            return QString("(%1, %2)").arg(*config.origin_pixel_x).arg(*config.origin_pixel_y);
        }

        return "(0, 0)";
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

    auto export_mirror_items() noexcept -> QVector<QString> {
        return { "左右镜像", "上下镜像", "不镜像" };
    }

    auto export_mirror_index(pcs::PngMapExportMirror mirror) noexcept -> int {
        switch (mirror) {
        case pcs::PngMapExportMirror::Horizontal:
            return 0;
        case pcs::PngMapExportMirror::Vertical:
            return 1;
        case pcs::PngMapExportMirror::None:
            return 2;
        }

        return 0;
    }

    auto export_mirror_from_index(int index) noexcept -> pcs::PngMapExportMirror {
        switch (index) {
        case 1:
            return pcs::PngMapExportMirror::Vertical;
        case 2:
            return pcs::PngMapExportMirror::None;
        default:
            return pcs::PngMapExportMirror::Horizontal;
        }
    }

    auto compact_dropdown_measurements() noexcept
        -> creeper::dropdown_menu::internal::DropdownMenu::Measurements {
        auto measurements = creeper::dropdown_menu::internal::DropdownMenu::Measurements {};

        measurements.container_height = 38;
        measurements.icon_rect_size = 16;
        measurements.input_rect_size = 16;
        measurements.label_rect_size = 12;
        measurements.standard_font_height = 13;
        measurements.col_padding = 6;
        measurements.row_padding_widthout_icons = 10;
        measurements.row_padding_with_icons = 8;
        measurements.row_padding_populated_label_text = 0;
        measurements.padding_icons_text = 8;
        measurements.supporting_text_and_character_counter_top_padding = 0;
        measurements.supporting_text_and_character_counter_row_padding = 0;

        return measurements;
    }

    class PngMapPanel final : public AssetActionPanel {
    public:
        explicit PngMapPanel(ActionPanelContext context, QFont const& font)
            : context { std::move(context) }
            , assets { *this->context.assets }
            , mouse { *this->context.mouse } {
            namespace ib = creeper::icon_button::pro;

            const auto theme        = creeper::theme::pro::ThemeManager { *this->context.manager };
            const auto current_tool = mouse.png_edit_tool();

            auto mode_label_font = font;
            mode_label_font.setPointSize(std::max(8, font.pointSize() - 1));

            const auto default_line_width = static_cast<qulonglong>(mouse.png_edit_line_width());
            const auto default_point_size = static_cast<qulonglong>(mouse.png_edit_point_size());
            const auto default_erase_size = static_cast<qulonglong>(mouse.png_edit_erase_size());

            auto* tool_row = new creeper::Row {
                creeper::row::pro::Spacing { 6 },
                creeper::row::pro::Alignment { Qt::AlignLeft },
            };
            for (auto const& descriptor :
                pcs::gui::interaction::default_png_edit_tool_descriptors()) {
                tool_row->addWidget(make_mode_item(theme, mode_label_font, current_tool,
                    descriptor.id, descriptor.label, descriptor.icon));
            }

            auto* free_param = new creeper::Widget {
                creeper::widget::pro::Layout<creeper::Col> {
                    creeper::col::pro::Spacing { 4 },
                    creeper::col::pro::Item<creeper::Text> {
                        theme,
                        creeper::text::pro::Font { font },
                        creeper::text::pro::Text {
                            "自由模式：用于查看当前坐标系下的位置，可拖动视图。" },
                        creeper::text::pro::WordWrap { true },
                        creeper::text::pro::Alignment { Qt::AlignLeft | Qt::AlignTop },
                    },
                },
            };

            line_width_row = new panels::CompactFieldRow(
                theme, font, "线宽（px）", 92, 116, QString::number(default_line_width));
            line_width_input = &line_width_row->field();
            auto* line_param = new creeper::Widget {
                creeper::widget::pro::Layout<creeper::Col> {
                    creeper::col::pro::Spacing { 4 },
                    creeper::col::pro::Item { line_width_row },
                },
            };

            point_size_row = new panels::CompactFieldRow(
                theme, font, "点大小（px）", 92, 116, QString::number(default_point_size));
            point_size_input  = &point_size_row->field();
            auto* point_param = new creeper::Widget {
                creeper::widget::pro::Layout<creeper::Col> {
                    creeper::col::pro::Spacing { 4 },
                    creeper::col::pro::Item { point_size_row },
                },
            };

            erase_size_row = new panels::CompactFieldRow(
                theme, font, "擦除大小（px）", 92, 116, QString::number(default_erase_size));
            erase_size_input  = &erase_size_row->field();
            auto* erase_param = new creeper::Widget {
                creeper::widget::pro::Layout<creeper::Col> {
                    creeper::col::pro::Spacing { 4 },
                    creeper::col::pro::Item { erase_size_row },
                },
            };

            param_stack = new creeper::Stacked {
                creeper::stacked::pro::Item { free_param },
                creeper::stacked::pro::Item { line_param },
                creeper::stacked::pro::Item { point_param },
                creeper::stacked::pro::Item { erase_param },
                creeper::stacked::pro::CurrentIndex {
                    pcs::gui::interaction::png_edit_tool_descriptor(current_tool).param_index },
            };

            const auto parameter_panel_height = std::max({
                free_param->sizeHint().height(),
                line_param->sizeHint().height(),
                point_param->sizeHint().height(),
                erase_param->sizeHint().height(),
                40,
            });
            auto* parameter_panel = new creeper::Widget {
                creeper::widget::pro::MinimumHeight { parameter_panel_height },
                creeper::widget::pro::Layout<creeper::Col> {
                    creeper::col::pro::Spacing { 0 },
                    creeper::col::pro::Item { param_stack },
                },
            };

            yaw_row   = new panels::AngleSliderFieldRow(theme, font, "Yaw", 84, 0.0);
            yaw_input = &yaw_row->field();
            QObject::connect(yaw_input, &creeper::OutlinedTextField::editingFinished,
                [this]() { sync_frame_config_into_asset(); });
            QObject::connect(&yaw_row->slider(), &creeper::Slider::signal_value_change,
                [this](double) { sync_frame_config_into_asset(); });

            set_origin_button = new creeper::OutlinedButton {
                theme,
                creeper::widget::pro::FixedHeight { 34 },
                creeper::button::pro::Text { "设置坐标系原点" },
            };
            QObject::connect(set_origin_button, &creeper::OutlinedButton::clicked, [this](bool) {
                if (selected_asset_id.empty()) {
                    return;
                }

                sync_frame_config_into_asset();
                const auto asset_id = selected_asset_id;
                mouse.set_selected_asset(selected_asset_id, pcs::AssetKind::PngMap);
                mouse.start_png_origin_pick({
                    .asset_id = asset_id,
                    .on_pick =
                        [this, asset_id](pcs::PixelPoint pixel) {
                            auto handle = assets.get_png_map_handle(asset_id);
                            if (!handle.has_value() || handle.value() == nullptr) {
                                return;
                            }

                            auto config           = handle.value()->get_frame_config();
                            config.origin_pixel_x = pixel.x;
                            config.origin_pixel_y = pixel.y;
                            handle.value()->set_frame_config(config);
                            refresh_origin_summary(config);
                            assets.update_renderer();
                            mouse.set_status(QString::fromUtf8("PNG 编辑 | 模式: %1 | 原点已设置为 "
                                                               "(%2, %3)")
                                    .arg(pcs::gui::interaction::png_edit_tool_descriptor(
                                        mouse.png_edit_tool())
                                            .label)
                                    .arg(pixel.x)
                                    .arg(pixel.y));
                        },
                    .on_cancel =
                        [this]() {
                            mouse.set_status(QString::fromUtf8("PNG 编辑 | 模式: %1")
                                    .arg(pcs::gui::interaction::png_edit_tool_descriptor(
                                        mouse.png_edit_tool())
                                            .label));
                        },
                });
                mouse.set_mode(pcs::gui::interaction::MouseModeId::PngOriginPick);
            });

            auto* mode_card = new creeper::FilledCard {
                theme,
                creeper::card::pro::LevelLowest,
                creeper::card::pro::Layout<creeper::Col> {
                    creeper::col::pro::Margin { 8 },
                    creeper::col::pro::Spacing { 8 },
                    creeper::col::pro::Item { tool_row },
                    creeper::col::pro::Item { parameter_panel },
                },
            };

            auto* frame_card = new creeper::FilledCard {
                theme,
                creeper::card::pro::LevelLowest,
                creeper::card::pro::Layout<creeper::Col> {
                    creeper::col::pro::Margin { 8 },
                    creeper::col::pro::Spacing { 8 },
                    creeper::col::pro::Item { yaw_row },
                    creeper::col::pro::Item { set_origin_button },
                },
            };

            export_button = new creeper::OutlinedButton {
                theme,
                creeper::widget::pro::FixedHeight { 36 },
                creeper::widget::pro::MinimumWidth { 180 },
                creeper::button::pro::Text { "导出" },
            };

            export_mirror_dropdown = new creeper::FilledDropdownMenu {
                theme,
                creeper::widget::pro::MinimumWidth { 150 },
                creeper::widget::pro::Apply { [](auto& self) {
                    self.set_measurements(compact_dropdown_measurements());
                } },
                creeper::filled_dropdown_menu::pro::Items { export_mirror_items() },
                creeper::filled_dropdown_menu::pro::IndexChanged { [this](int index) {
                    if (index >= 0) {
                        sync_frame_config_into_asset();
                    }
                } },
            };
            export_mirror_dropdown->setCurrentIndex(0);

            auto* export_option_row = new creeper::Row {
                creeper::row::pro::Spacing { 8 },
                creeper::row::pro::Item<creeper::Text> {
                    { 0, Qt::AlignVCenter },
                    theme,
                    creeper::text::pro::Font { font },
                    creeper::text::pro::Text { "导出镜像" },
                    creeper::widget::pro::FixedWidth { 92 },
                },
                creeper::row::pro::Item { { 1, Qt::AlignVCenter }, export_mirror_dropdown },
            };

            auto* export_card = new creeper::FilledCard {
                theme,
                creeper::card::pro::LevelLowest,
                creeper::card::pro::Layout<creeper::Col> {
                    creeper::col::pro::Margin { 8 },
                    creeper::col::pro::Spacing { 8 },
                    creeper::col::pro::Item { export_option_row },
                    creeper::col::pro::Item { export_button },
                },
            };

            QObject::connect(export_button, &creeper::OutlinedButton::clicked, [this](bool) {
                if (selected_asset_id.empty()) {
                    return;
                }

                sync_frame_config_into_asset();

                const auto suggested_name =
                    assets.get_asset_name(selected_asset_id).value_or("map");
                if (auto location = panels::export_png_map_directory(suggested_name)) {
                    const auto result = assets.export_png_map_asset(selected_asset_id, *location);
                    if (!result.has_value()) {
                        spdlog::error("导出 PNG 地图资产失败: {}", result.error());
                        return;
                    }

                    this->context.refresh_assets_list();
                    this->context.select_asset(selected_asset_id);
                }
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
                        creeper::text::pro::Text { "PNG 地图操作" },
                        creeper::text::pro::Alignment { Qt::AlignHCenter },
                    },
                    creeper::col::pro::Item { mode_card },
                    creeper::col::pro::Item { frame_card },
                    creeper::col::pro::Item { export_card },
                },
            };
        }

        auto widget() const noexcept -> QWidget* override { return root; }

        auto bind_asset(std::string const& id) noexcept -> void override {
            selected_asset_id = id;

            sync_inputs_into_mouse();
            mouse.set_selected_asset(id, pcs::AssetKind::PngMap);
            mouse.set_mode(pcs::gui::interaction::MouseModeId::PngEdit);

            if (auto handle = assets.get_png_map_handle(id);
                handle.has_value() && handle.value() != nullptr) {
                const auto config = handle.value()->get_frame_config();
                yaw_row->set_degrees(config.yaw_deg);
                export_mirror_dropdown->setCurrentIndex(export_mirror_index(config.export_mirror));
                refresh_origin_summary(config);
            }

            mouse.set_status(QString("PNG 编辑 | 模式: %1")
                    .arg(pcs::gui::interaction::png_edit_tool_descriptor(mouse.png_edit_tool())
                            .label));

            sync_tool_buttons();
        }

        auto clear() noexcept -> void override {
            selected_asset_id.clear();

            if (mouse.mode() == pcs::gui::interaction::MouseModeId::PngEdit) {
                mouse.set_mode(pcs::gui::interaction::MouseModeId::None);
            }
        }

    private:
        auto sync_frame_config_into_asset() noexcept -> void {
            if (selected_asset_id.empty()) {
                return;
            }

            auto handle = assets.get_png_map_handle(selected_asset_id);
            if (!handle.has_value() || handle.value() == nullptr) {
                return;
            }

            auto config    = handle.value()->get_frame_config();
            config.yaw_deg = yaw_row->degrees();
            config.export_mirror = export_mirror_from_index(export_mirror_dropdown->currentIndex());
            if (handle.value()->set_frame_config(config)) {
                refresh_origin_summary(config);
                assets.update_renderer();
            }
        }

        auto make_mode_item(creeper::theme::pro::ThemeManager const& theme,
            QFont const& mode_label_font, pcs::gui::interaction::PngEditTool current_tool,
            pcs::gui::interaction::PngEditTool tool, QString const& label,
            QString const& icon) noexcept -> QWidget* {
            namespace ib = creeper::icon_button::pro;

            auto* button = new creeper::IconButton {
                ib::ThemeManager { *context.manager },
                ib::ColorFilled,
                ib::ShapeRound,
                ib::WidthDefault,
                ib::FixedSize { creeper::IconButton::kExtraSmallContainerSize },
                ib::Font {
                    creeper::material::round::font, creeper::IconButton::kExtraSmallFontIconSize },
                tool == current_tool ? ib::TypesToggleSelected : ib::TypesToggleUnselected,
                ib::FontIcon { icon },
                ib::ToolTip { QString("模式: %1").arg(label) },
                ib::Clickable { [this, tool] {
                    if (selected_asset_id.empty()) {
                        return;
                    }

                    sync_inputs_into_mouse();
                    mouse.set_selected_asset(selected_asset_id, pcs::AssetKind::PngMap);
                    mouse.set_png_edit_tool(tool);
                    mouse.set_mode(pcs::gui::interaction::MouseModeId::PngEdit);
                    mouse.set_status(
                        QString("PNG 编辑 | 模式: %1").arg(png_edit_tool_descriptor(tool).label));
                    sync_tool_buttons();
                } },
            };

            mode_buttons.push_back(button);
            mode_button_tools.push_back(tool);

            return new creeper::Widget {
                creeper::widget::pro::Layout<creeper::Col> {
                    creeper::col::pro::Spacing { 2 },
                    creeper::col::pro::Alignment { Qt::AlignHCenter },
                    creeper::col::pro::Item { { 0, Qt::AlignHCenter }, button },
                    creeper::col::pro::Item<creeper::Text> {
                        { 0, Qt::AlignHCenter },
                        theme,
                        creeper::text::pro::Font { mode_label_font },
                        creeper::text::pro::Text { label },
                        creeper::text::pro::Alignment { Qt::AlignHCenter },
                        creeper::widget::pro::FixedWidth {
                            creeper::IconButton::kExtraSmallContainerSize.width() },
                    },
                },
            };
        }

        auto refresh_origin_summary(pcs::PngMapFrameConfig const& config) noexcept -> void {
            set_origin_button->setText(
                QString::fromUtf8("设置坐标系原点 %1").arg(origin_summary(config)));
        }

        auto sync_inputs_into_mouse() noexcept -> void {
            const auto line_width =
                panels::parse_size_input(*line_width_input, mouse.png_edit_line_width(), 1);
            const auto point_size =
                panels::parse_size_input(*point_size_input, mouse.png_edit_point_size(), 1);
            const auto erase_size =
                panels::parse_size_input(*erase_size_input, mouse.png_edit_erase_size(), 1);

            mouse.set_png_edit_line_width(line_width);
            mouse.set_png_edit_point_size(point_size);
            mouse.set_png_edit_erase_size(erase_size);
        }

        auto sync_tool_buttons() noexcept -> void {
            const auto current_tool = mouse.png_edit_tool();
            for (std::size_t i = 0; i < mode_buttons.size() && i < mode_button_tools.size(); ++i) {
                mode_buttons[i]->set_selected(mode_button_tools[i] == current_tool);
            }

            param_stack->setCurrentIndex(
                pcs::gui::interaction::png_edit_tool_descriptor(current_tool).param_index);
        }

        ActionPanelContext context;
        pcs::AssetsManager& assets;
        pcs::gui::interaction::Mouse& mouse;

        std::string selected_asset_id;

        std::vector<creeper::IconButton*> mode_buttons;
        std::vector<pcs::gui::interaction::PngEditTool> mode_button_tools;

        panels::CompactFieldRow* line_width_row = nullptr;
        panels::CompactFieldRow* point_size_row = nullptr;
        panels::CompactFieldRow* erase_size_row = nullptr;
        panels::AngleSliderFieldRow* yaw_row    = nullptr;

        creeper::OutlinedTextField* line_width_input = nullptr;
        creeper::OutlinedTextField* point_size_input = nullptr;
        creeper::OutlinedTextField* erase_size_input = nullptr;
        creeper::OutlinedTextField* yaw_input        = nullptr;

        creeper::Stacked* param_stack = nullptr;

        creeper::OutlinedButton* set_origin_button = nullptr;
        creeper::FilledDropdownMenu* export_mirror_dropdown = nullptr;
        creeper::OutlinedButton* export_button     = nullptr;
        creeper::FilledCard* root                  = nullptr;
    };

}

auto make_png_map_panel(ActionPanelContext context, QFont const& font)
    -> std::unique_ptr<AssetActionPanel> {
    return std::make_unique<PngMapPanel>(std::move(context), font);
}

}
