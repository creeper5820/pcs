#include "gui/navigation.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/layout/mutual-exclusion-group.hh>
#include <creeper-qt/utility/material-icon.hh>
#include <creeper-qt/widget/buttons/icon-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/cards/outlined-card.hh>
#include <creeper-qt/widget/image.hh>

#include <qpointer.h>

using namespace creeper;

auto NavigationComponent(NavigationState& state) noexcept -> QPointer<QWidget> {
    namespace im = image::pro;
    namespace ib = icon_button::pro;

    const auto common_button = std::tuple {
        ib::ThemeManager { state.manager },
        ib::TypesDefault,
        ib::ShapeRound,
        ib::WidthDefault,
        ib::ColorStandard,
        ib::FixedSize { IconButton::kSmallContainerSize },
        ib::Font { state.icon_font.c_str(), IconButton::kSmallFontIconSize },
    };

    const auto picker_enabled = [&state] {
        if (state.picker_mode_getter) {
            return state.picker_mode_getter();
        }
        return false;
    };

    const auto picker_button_type =
        picker_enabled() ? ib::TypesToggleSelected : ib::TypesToggleUnselected;

    auto picker_button = new IconButton {
        common_button,
        picker_button_type,
        ib::ColorFilled,
        ib::FontIcon { "ads_click" },
        ib::ToolTip { "切换坐标拾取模式" },
        ib::Clickable { [&](IconButton& self) {
            if (!state.picker_mode_getter || !state.picker_mode_setter) {
                return;
            }

            const auto next = !state.picker_mode_getter();
            state.picker_mode_setter(next);
            self.set_selected(next);
        } },
    };

    if (state.mouse != nullptr) {
        state.mouse->set_mode_sink([guard = QPointer<IconButton> { picker_button }](
                                       ::pcs::gui::interaction::MouseModeId mode) {
            if (guard != nullptr) {
                guard->set_selected(mode == ::pcs::gui::interaction::MouseModeId::Picker);
            }
        });
    }

    const auto AvatarComponent = new Image {
        im::FixedSize { 60, 60 },
        im::Radius { -1 },
        im::ContentScale { ContentScale::CROP },
        im::BorderWidth { 3 },
        im::PainterResource {
            "http://i0.hdslb.com/bfs/article/e4e412299e6c038035241b1dc625cb62c8b5513a.jpg",
        },
    };
    state.manager.append_handler(AvatarComponent, [AvatarComponent](const auto& manager) {
        const auto colorscheme = manager.color_scheme();
        const auto colorborder = colorscheme.secondary_container;
        AvatarComponent->set_border_color(colorborder);
    });

    return new FilledCard {
        card::pro::ThemeManager { state.manager },
        card::pro::LevelLow,
        card::pro::FixedWidth { 80 },
        card::pro::Radius { 0 },

        card::pro::Layout<Col> {
            col::pro::ContentsMargin { { 5, 10, 5, 10 } },
            col::pro::Alignment { Qt::AlignHCenter },

            col::pro::SpacingItem { 20 },
            col::pro::Item {
                { 0, Qt::AlignHCenter },
                AvatarComponent,
            },
            col::pro::SpacingItem { 20 },

            col::pro::Item<SelectGroup<Col, IconButton>> {
                { 0, Qt::AlignHCenter },
                col::pro::Spacing { 10 },
                col::pro::Alignment { Qt::AlignHCenter },
                select_group::pro::Compose {
                    state.buttons_context,
                    [&](NavigationState::ButtonContext const& context) {
                        return new IconButton {
                            common_button,
                            ib::FontIcon { context.icon.data() },
                            ib::Clickable { context.callback },
                        };
                    },
                },
            },
            col::pro::Item { { 0, Qt::AlignHCenter }, picker_button },

            col::pro::Stretch { 255 },

            col::pro::Item<IconButton> {
                { 0, Qt::AlignHCenter },
                common_button,
                ib::FontIcon { material::icon::kLogout },
                ib::Clickable { [&] { state.function_quit(); } },
            },
            col::pro::Item<IconButton> {
                { 0, Qt::AlignHCenter },
                common_button,
                ib::ColorFilled,
                ib::TypesToggleUnselected,
                ib::FontIcon { material::icon::kDarkMode },
                ib::Clickable { [&] {
                    state.manager.toggle_color_mode();
                    state.manager.apply_theme();
                } },
            },
            col::pro::SpacingItem { 20 },
        },
    };
}
