#include "gui/navigation.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/layout/mutual-exclusion-group.hh>
#include <creeper-qt/utility/material-icon.hh>
#include <creeper-qt/widget/buttons/icon-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/cards/outlined-card.hh>
#include <creeper-qt/widget/image.hh>

#include <QPointer>
#include <QVBoxLayout>

using namespace creeper;

struct Navigation::Impl {
    ThemeManager& manager;
    std::string icon_font;
    pcs::gui::interaction::Mouse& mouse;
    std::vector<ButtonContext> buttons_context;
    std::function<void(bool)> picker_mode_setter;
    std::function<bool()> picker_mode_getter;
    std::function<void()> function_quit;

    explicit Impl(ThemeManager& manager, std::string icon_font,
        std::vector<ButtonContext> buttons_context, std::function<void(bool)> picker_mode_setter,
        std::function<bool()> picker_mode_getter, std::function<void()> function_quit,
        pcs::gui::interaction::Mouse& mouse) noexcept
        : manager { manager }
        , icon_font { std::move(icon_font) }
        , mouse { mouse }
        , buttons_context { std::move(buttons_context) }
        , picker_mode_setter { std::move(picker_mode_setter) }
        , picker_mode_getter { std::move(picker_mode_getter) }
        , function_quit { std::move(function_quit) } { }
};

Navigation::Navigation(ThemeManager& manager, std::string icon_font,
    std::vector<ButtonContext> buttons_context, std::function<void(bool)> picker_mode_setter,
    std::function<bool()> picker_mode_getter, std::function<void()> function_quit,
    pcs::gui::interaction::Mouse& mouse) noexcept
    : pimpl { std::make_unique<Impl>(manager, std::move(icon_font), std::move(buttons_context),
          std::move(picker_mode_setter), std::move(picker_mode_getter), std::move(function_quit),
          mouse) } {
    namespace im = image::pro;
    namespace ib = icon_button::pro;

    const auto common_button = std::tuple {
        ib::ThemeManager { pimpl->manager },
        ib::TypesDefault,
        ib::ShapeRound,
        ib::WidthDefault,
        ib::ColorStandard,
        ib::FixedSize { IconButton::kSmallContainerSize },
        ib::Font { pimpl->icon_font.c_str(), IconButton::kSmallFontIconSize },
    };

    const auto picker_enabled = [this] {
        if (pimpl->picker_mode_getter) {
            return pimpl->picker_mode_getter();
        }
        return false;
    };

    const auto picker_button_type =
        picker_enabled() ? ib::TypesToggleSelected : ib::TypesToggleUnselected;

    auto* picker_button = new IconButton {
        common_button,
        picker_button_type,
        ib::ColorFilled,
        ib::FontIcon { "ads_click" },
        ib::ToolTip { "切换坐标拾取模式" },
        ib::Clickable { [this](IconButton& self) {
            if (!pimpl->picker_mode_getter || !pimpl->picker_mode_setter) {
                return;
            }

            const auto next = !pimpl->picker_mode_getter();
            pimpl->picker_mode_setter(next);
            self.set_selected(next);
        } },
    };

    pimpl->mouse.set_mode_sink([guard = QPointer<IconButton> { picker_button }](
                                   ::pcs::gui::interaction::MouseModeId mode) {
        if (guard != nullptr) {
            guard->set_selected(mode == ::pcs::gui::interaction::MouseModeId::Picker);
        }
    });

    auto* avatar_component = new Image {
        im::FixedSize { 60, 60 },
        im::Radius { -1 },
        im::ContentScale { ContentScale::CROP },
        im::BorderWidth { 3 },
        im::PainterResource {
            "http://i0.hdslb.com/bfs/article/e4e412299e6c038035241b1dc625cb62c8b5513a.jpg",
        },
    };
    pimpl->manager.append_handler(avatar_component,
        [avatar_component](const auto& manager) {
            const auto colorscheme = manager.color_scheme();
            const auto colorborder = colorscheme.secondary_container;
            avatar_component->set_border_color(colorborder);
        });

    auto* root = new FilledCard {
        card::pro::ThemeManager { pimpl->manager },
        card::pro::LevelLow,
        card::pro::FixedWidth { 80 },
        card::pro::Radius { 0 },

        card::pro::Layout<Col> {
            col::pro::ContentsMargin { { 5, 10, 5, 10 } },
            col::pro::Alignment { Qt::AlignHCenter },

            col::pro::SpacingItem { 20 },
            col::pro::Item {
                { 0, Qt::AlignHCenter },
                avatar_component,
            },
            col::pro::SpacingItem { 20 },

            col::pro::Item<SelectGroup<Col, IconButton>> {
                { 0, Qt::AlignHCenter },
                col::pro::Spacing { 10 },
                col::pro::Alignment { Qt::AlignHCenter },
                select_group::pro::Compose {
                    pimpl->buttons_context,
                    [&](Navigation::ButtonContext const& context) {
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
                ib::Clickable { [this] { pimpl->function_quit(); } },
            },
            col::pro::Item<IconButton> {
                { 0, Qt::AlignHCenter },
                common_button,
                ib::ColorFilled,
                ib::TypesToggleUnselected,
                ib::FontIcon { material::icon::kDarkMode },
                ib::Clickable { [this] {
                    pimpl->manager.toggle_color_mode();
                    pimpl->manager.apply_theme();
                } },
            },
            col::pro::SpacingItem { 20 },
        },
    };

    auto* layout = new QVBoxLayout { };
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(root);
    setLayout(layout);
}

Navigation::~Navigation() noexcept = default;
