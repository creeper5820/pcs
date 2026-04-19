#include "gui/working/panels/png-map-panel.hh"

#include "gui/interaction/png-edit-tools.hh"
#include "gui/working/panels/common.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/layout/stacked.hh>
#include <creeper-qt/utility/material-icon.hh>
#include <creeper-qt/widget/buttons/icon-button.hh>
#include <creeper-qt/widget/buttons/outlined-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/text.hh>
#include <creeper-qt/widget/widget.hh>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <tuple>
#include <vector>

namespace pcs::gui::working {

namespace {

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

            const auto default_line_width = static_cast<qulonglong>(mouse.png_edit_line_width());
            const auto default_point_size = static_cast<qulonglong>(mouse.png_edit_point_size());
            const auto default_erase_size = static_cast<qulonglong>(mouse.png_edit_erase_size());

            const auto mode_button_base = std::tuple {
                ib::ThemeManager { *this->context.manager },
                ib::ColorFilled,
                ib::ShapeRound,
                ib::WidthDefault,
                ib::FixedSize { creeper::IconButton::kExtraSmallContainerSize },
                ib::Font {
                    creeper::material::round::font, creeper::IconButton::kExtraSmallFontIconSize },
            };

            const auto make_mode_item = [this, theme, mode_label_font, mode_button_base,
                                            current_tool](pcs::gui::interaction::PngEditTool tool,
                                            QString const& label, QString const& icon) -> QWidget* {
                namespace ib = creeper::icon_button::pro;

                auto* button = new creeper::IconButton {
                    mode_button_base,
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
            };

            auto* tool_row = new creeper::Row {
                creeper::row::pro::Spacing { 6 },
                creeper::row::pro::Alignment { Qt::AlignHCenter },
            };
            for (auto const& descriptor : pcs::gui::interaction::default_png_edit_tool_descriptors()) {
                tool_row->addWidget(
                    make_mode_item(descriptor.id, descriptor.label, descriptor.icon));
            }

            auto* free_param = new creeper::Text {
                theme,
                creeper::text::pro::Font { font },
                creeper::text::pro::Text { "自由模式：用于查看坐标，可拖动视图" },
                creeper::text::pro::WordWrap { true },
            };

            auto* line_param = new creeper::Widget {
                creeper::widget::pro::Layout<creeper::Col> {
                    creeper::col::pro::Spacing { 4 },
                    creeper::col::pro::Item { make_input_row(
                        "线宽（px）", line_width_input, QString::number(default_line_width)) },
                },
            };

            auto* point_param = new creeper::Widget {
                creeper::widget::pro::Layout<creeper::Col> {
                    creeper::col::pro::Spacing { 4 },
                    creeper::col::pro::Item { make_input_row(
                        "点大小（px）", point_size_input, QString::number(default_point_size)) },
                },
            };

            auto* erase_param = new creeper::Widget {
                creeper::widget::pro::Layout<creeper::Col> {
                    creeper::col::pro::Spacing { 4 },
                    creeper::col::pro::Item { make_input_row(
                        "擦除大小（px）", erase_size_input, QString::number(default_erase_size)) },
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

            save_button = new creeper::OutlinedButton {
                theme,
                creeper::widget::pro::FixedHeight { 36 },
                creeper::widget::pro::MinimumWidth { 180 },
                creeper::button::pro::Text { "保存 PNG 地图" },
            };

            QObject::connect(save_button, &creeper::OutlinedButton::clicked, [this](bool) {
                if (selected_asset_id.empty()) {
                    return;
                }

                const auto suggested_name =
                    assets.get_asset_name(selected_asset_id).value_or("map.png");
                if (auto location = panels::save_png_map_location(suggested_name)) {
                    const auto result = assets.save_png_map_asset(selected_asset_id, *location);
                    if (!result.has_value()) {
                        spdlog::error("保存 PNG 地图资产失败: {}", result.error());
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
                    creeper::col::pro::Item<creeper::FilledCard> {
                        theme,
                        creeper::card::pro::LevelLowest,
                        creeper::card::pro::Layout<creeper::Col> {
                            creeper::col::pro::Margin { 8 },
                            creeper::col::pro::Spacing { 6 },
                            creeper::col::pro::Item { tool_row },
                        },
                    },
                    creeper::col::pro::Item<creeper::FilledCard> {
                        theme,
                        creeper::card::pro::LevelLowest,
                        creeper::card::pro::Layout<creeper::Col> {
                            creeper::col::pro::Margin { 8 },
                            creeper::col::pro::Spacing { 4 },
                            creeper::col::pro::Item { param_stack },
                        },
                    },
                    creeper::col::pro::Item { save_button },
                },
            };
        }

        auto widget() const noexcept -> QWidget* override { return root; }

        auto bind_asset(std::string const& id) noexcept -> void override {
            selected_asset_id = id;

            sync_inputs_into_mouse();
            mouse.set_selected_asset(id, pcs::AssetKind::PngMap);
            mouse.set_mode(pcs::gui::interaction::MouseModeId::PngEdit);
            mouse.set_status(QString("PNG 编辑 | 模式: %1")
                                 .arg(pcs::gui::interaction::png_edit_tool_descriptor(
                                     mouse.png_edit_tool())
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

        creeper::OutlinedTextField* line_width_input = nullptr;
        creeper::OutlinedTextField* point_size_input = nullptr;
        creeper::OutlinedTextField* erase_size_input = nullptr;

        creeper::Stacked* param_stack = nullptr;

        creeper::OutlinedButton* save_button = nullptr;
        creeper::FilledCard* root            = nullptr;
    };

}

auto make_png_map_panel(ActionPanelContext context, QFont const& font)
    -> std::unique_ptr<AssetActionPanel> {
    return std::make_unique<PngMapPanel>(std::move(context), font);
}

}
