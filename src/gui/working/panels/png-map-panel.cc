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
#include <creeper-qt/widget/switch.hh>
#include <creeper-qt/widget/text.hh>
#include <creeper-qt/widget/widget.hh>

#include <QSignalBlocker>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <vector>

using namespace creeper;
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
        auto measurements = creeper::dropdown_menu::internal::DropdownMenu::Measurements { };

        measurements.container_height                                  = 38;
        measurements.icon_rect_size                                    = 16;
        measurements.input_rect_size                                   = 16;
        measurements.label_rect_size                                   = 12;
        measurements.standard_font_height                              = 13;
        measurements.col_padding                                       = 6;
        measurements.row_padding_widthout_icons                        = 10;
        measurements.row_padding_with_icons                            = 8;
        measurements.row_padding_populated_label_text                  = 0;
        measurements.padding_icons_text                                = 8;
        measurements.supporting_text_and_character_counter_top_padding = 0;
        measurements.supporting_text_and_character_counter_row_padding = 0;

        return measurements;
    }

    class PngMapPanelBody final : public AssetActionPanel {
    public:
        explicit PngMapPanelBody(ActionPanelContext context, QFont const& font)
            : context { std::move(context) }
            , assets { *this->context.assets }
            , mouse { *this->context.mouse } {
            using creeper::Col;
            using creeper::FilledCard;
            using creeper::OutlinedButton;
            using creeper::Row;
            using creeper::Stacked;
            using creeper::Text;
            using creeper::Widget;

            namespace ib = creeper::icon_button::pro;
            namespace ob = creeper::outlined_button::pro;

            const auto theme        = creeper::theme::pro::ThemeManager { *this->context.manager };
            const auto current_tool = mouse.png_edit_tool();

            auto mode_label_font = font;
            mode_label_font.setPointSize(std::max(8, font.pointSize() - 1));

            const auto default_line_width = static_cast<qulonglong>(mouse.png_edit_line_width());
            const auto default_point_size = static_cast<qulonglong>(mouse.png_edit_point_size());
            const auto default_erase_size = static_cast<qulonglong>(mouse.png_edit_erase_size());

            auto* tool_row = new Row {
                row::pro::Spacing { 6 },
                row::pro::Alignment { Qt::AlignLeft },
            };
            for (auto const& descriptor :
                pcs::gui::interaction::default_png_edit_tool_descriptors()) {
                tool_row->addWidget(make_mode_item(theme, mode_label_font, current_tool,
                    descriptor.id, descriptor.label, descriptor.icon));
            }

            auto* free_param = new Widget {
                widget::pro::Layout<Col> {
                    col::pro::Spacing { 4 },
                    col::pro::Item<Text> {
                        theme,
                        text::pro::Font { font },
                        text::pro::Text { "自由模式：用于查看当前坐标系下的位置，可拖动视图。" },
                        text::pro::WordWrap { true },
                        text::pro::Alignment { Qt::AlignLeft | Qt::AlignTop },
                    },
                },
            };

            line_width_row = new panels::CompactFieldRow(
                theme, font, "线宽（px）", 92, 116, QString::number(default_line_width));
            line_width_input = &line_width_row->field();
            auto* line_param = new Widget {
                widget::pro::Layout<Col> {
                    col::pro::Spacing { 4 },
                    col::pro::Item { line_width_row },
                },
            };

            point_size_row = new panels::CompactFieldRow(
                theme, font, "点大小（px）", 92, 116, QString::number(default_point_size));
            point_size_input  = &point_size_row->field();
            auto* point_param = new Widget {
                widget::pro::Layout<Col> {
                    col::pro::Spacing { 4 },
                    col::pro::Item { point_size_row },
                },
            };

            erase_size_row = new panels::CompactFieldRow(
                theme, font, "擦除大小（px）", 92, 116, QString::number(default_erase_size));
            erase_size_input  = &erase_size_row->field();
            auto* erase_param = new Widget {
                widget::pro::Layout<Col> {
                    col::pro::Spacing { 4 },
                    col::pro::Item { erase_size_row },
                },
            };

            param_stack = new Stacked {
                stacked::pro::Item { free_param },
                stacked::pro::Item { line_param },
                stacked::pro::Item { point_param },
                stacked::pro::Item { erase_param },
                stacked::pro::CurrentIndex {
                    pcs::gui::interaction::png_edit_tool_descriptor(current_tool).param_index },
            };

            const auto parameter_panel_height = std::max({
                free_param->sizeHint().height(),
                line_param->sizeHint().height(),
                point_param->sizeHint().height(),
                erase_param->sizeHint().height(),
                40,
            });
            yaw_row   = new panels::AngleSliderFieldRow(theme, font, "Yaw", 84, 0.0);
            yaw_input = &yaw_row->field();
            QObject::connect(yaw_input, &OutlinedTextField::editingFinished,
                [this]() { sync_frame_config_into_asset(); });
            QObject::connect(&yaw_row->slider(), &Slider::signal_value_change,
                [this](double) { sync_frame_config_into_asset(); });

            namespace sw       = creeper::_switch::pro;
            source_area_switch = new creeper::Switch {
                sw::ThemeManager { *this->context.manager },
                sw::Checked { true },
                sw::FixedSize { 44, 24 },
                sw::Clickable { [this] {
                    if (selected_asset_id.empty()) {
                        return;
                    }

                    auto handle = assets.get_png_map_handle(selected_asset_id);
                    if (!handle.has_value() || handle.value() == nullptr) {
                        return;
                    }

                    handle.value()->set_source_area_visibility(source_area_switch->isChecked());
                    assets.update_renderer();
                } },
            };
            source_area_switch_row =
                new panels::CompactWidgetRow(theme, font, "显示源点云区域", 92, source_area_switch);

            set_origin_button = new OutlinedButton {
                theme,
                ob::FixedHeight { 34 },
                ob::Text { "设置坐标系原点" },
                ob::Clickable { [this] {
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
                                mouse.set_status(QString::fromUtf8("PNG 编辑 | 模式: %1 | "
                                                                   "原点已设置为 "
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
                } },
            };

            export_mirror_dropdown = new creeper::FilledDropdownMenu {
                theme,
                widget::pro::MinimumWidth { 150 },
                widget::pro::Apply {
                    [](auto& self) { self.set_measurements(compact_dropdown_measurements()); } },
                filled_dropdown_menu::pro::Items { export_mirror_items() },
                filled_dropdown_menu::pro::IndexChanged { [this](int index) {
                    if (index >= 0) {
                        sync_frame_config_into_asset();
                    }
                } },
            };
            export_mirror_dropdown->setCurrentIndex(0);

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
                        text::pro::Text { "PNG 地图操作" },
                        text::pro::Alignment { Qt::AlignHCenter },
                    },
                    col::pro::Item<FilledCard> {
                        theme,
                        card::pro::LevelLowest,
                        card::pro::Layout<Col> {
                            col::pro::Margin { 8 },
                            col::pro::Spacing { 8 },
                            col::pro::Item { tool_row },
                            col::pro::Item<Widget> {
                                widget::pro::MinimumHeight { parameter_panel_height },
                                widget::pro::Layout<Col> {
                                    col::pro::Spacing { 0 },
                                    col::pro::Item { param_stack },
                                },
                            },
                        },
                    },
                    col::pro::Item<FilledCard> {
                        theme,
                        card::pro::LevelLowest,
                        card::pro::Layout<Col> {
                            col::pro::Margin { 8 },
                            col::pro::Spacing { 8 },
                            col::pro::Item { source_area_switch_row },
                            col::pro::Item { yaw_row },
                            col::pro::Item { set_origin_button },
                        },
                    },
                    col::pro::Item<FilledCard> {
                        theme,
                        card::pro::LevelLowest,
                        card::pro::Layout<Col> {
                            col::pro::Margin { 8 },
                            col::pro::Spacing { 8 },
                            col::pro::Item<panels::CompactWidgetRow> {
                                theme,
                                font,
                                "导出镜像",
                                92,
                                export_mirror_dropdown,
                            },
                            col::pro::Item<OutlinedButton> {
                                theme,
                                ob::FixedHeight { 36 },
                                ob::MinimumWidth { 180 },
                                ob::Text { "导出" },
                                ob::Clickable { [this] {
                                    if (selected_asset_id.empty()) {
                                        return;
                                    }

                                    sync_frame_config_into_asset();

                                    const auto suggested_name =
                                        assets.get_asset_name(selected_asset_id).value_or("map");
                                    if (auto location =
                                            panels::export_png_map_directory(suggested_name)) {
                                        const auto result = assets.export_png_map_asset(
                                            selected_asset_id, *location);
                                        if (!result.has_value()) {
                                            spdlog::error(
                                                "导出 PNG 地图资产失败: {}", result.error());
                                            return;
                                        }

                                        this->context.refresh_assets_list();
                                        this->context.select_asset(selected_asset_id);
                                    }
                                } },
                            },
                        },
                    },
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
                if (source_area_switch != nullptr) {
                    const auto blocker = QSignalBlocker { source_area_switch };
                    source_area_switch->setChecked(handle.value()->source_area_visibility());
                }
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

            auto config          = handle.value()->get_frame_config();
            config.yaw_deg       = yaw_row->degrees();
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
                widget::pro::Layout<creeper::Col> {
                    col::pro::Spacing { 2 },
                    col::pro::Alignment { Qt::AlignHCenter },
                    col::pro::Item { { 0, Qt::AlignHCenter }, button },
                    col::pro::Item<creeper::Text> {
                        { 0, Qt::AlignHCenter },
                        theme,
                        text::pro::Font { mode_label_font },
                        text::pro::Text { label },
                        text::pro::Alignment { Qt::AlignHCenter },
                        widget::pro::FixedWidth {
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

        panels::CompactFieldRow* line_width_row          = nullptr;
        panels::CompactFieldRow* point_size_row          = nullptr;
        panels::CompactFieldRow* erase_size_row          = nullptr;
        panels::AngleSliderFieldRow* yaw_row             = nullptr;
        panels::CompactWidgetRow* source_area_switch_row = nullptr;

        creeper::OutlinedTextField* line_width_input = nullptr;
        creeper::OutlinedTextField* point_size_input = nullptr;
        creeper::OutlinedTextField* erase_size_input = nullptr;
        creeper::OutlinedTextField* yaw_input        = nullptr;
        creeper::Switch* source_area_switch          = nullptr;

        creeper::Stacked* param_stack = nullptr;

        creeper::OutlinedButton* set_origin_button          = nullptr;
        creeper::FilledDropdownMenu* export_mirror_dropdown = nullptr;
        creeper::FilledCard* root                           = nullptr;
    };

}

struct PngMapPanel::Impl {
    std::unique_ptr<AssetActionPanel> panel;
};

PngMapPanel::~PngMapPanel() noexcept = default;

auto PngMapPanel::widget() const noexcept -> QWidget* { return pimpl->panel->widget(); }

auto PngMapPanel::bind_asset(std::string const& id) noexcept -> void {
    pimpl->panel->bind_asset(id);
}

auto PngMapPanel::clear() noexcept -> void { pimpl->panel->clear(); }

PngMapPanel::PngMapPanel(ActionPanelContext context, QFont const& font) noexcept
    : pimpl { std::make_unique<Impl>() } {
    pimpl->panel = std::make_unique<PngMapPanelBody>(std::move(context), font);
}

}
