#include "side-toolbar.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/utility/material-icon.hh>
#include <creeper-qt/widget/buttons/icon-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>

#include <qapplication.h>
#include <qfiledialog.h>
#include <qtimer.h>
#include <spdlog/spdlog.h>

using namespace gui;
using namespace creeper;

namespace a = card::pro;
namespace b = linear::pro;
namespace c = icon_button::pro;

struct SideToolbar::Impl {
    FilledCard* container;

    explicit Impl(ThemeManager& manager) noexcept {

        const auto icon_button_properties = std::tuple {
            c::ThemeManager { manager },
            c::Types { IconButton::Types::DEFAULT },
            c::Color { IconButton::Color::STANDARD },
            c::Width { IconButton::Width::WIDE },
            c::FixedSize { IconButton::kMediumContainerSize },
            c::Font { material::kRoundMediumFont },
        };
        container = new FilledCard {
            a::Radius { 0 },
            a::Layout<Col> {
                b::Alignment { Qt::AlignTop },
                b::Margin { 10 },
                b::Spacing { 0 },
                b::Item<IconButton> {
                    { 1, Qt::AlignLeft },
                    icon_button_properties,
                    c::FontIcon { "menu" },
                },
                b::Spacing { 20 },
                b::Item<IconButton> {
                    { 1, Qt::AlignLeft },
                    icon_button_properties,
                    c::FontIcon { "cloud_download" },
                },
                b::Item<IconButton> {
                    { 1, Qt::AlignLeft },
                    icon_button_properties,
                    c::FontIcon { "delete_sweep" },
                },
                b::Stretch { 255 },
                b::Item<IconButton> {
                    { 1, Qt::AlignLeft },
                    icon_button_properties,
                    c::Types { IconButton::Types::DEFAULT },
                    c::FontIcon { "exit_to_app" },
                    c::Clickable { [](auto&) { QApplication::exit(); } },
                },
                b::Item<IconButton> {
                    { 1, Qt::AlignLeft },
                    icon_button_properties,
                    c::FontIcon { "dark_mode" },
                    c::Types { IconButton::Types::TOGGLE_UNSELECTED },
                    c::Color { IconButton::Color::DEFAULT_FILLED },
                    c::Clickable { [&](IconButton& self) {
                        manager.toggle_color_mode();
                        manager.apply_theme();
                    } },
                },
            },
        };

        manager.append_handler(container, [this](const ThemeManager& manager) {
            {
                const auto color = manager.color_scheme().secondary_container;
                container->set_background(color);
            }
            {
                const auto color = manager.color_scheme().primary;
                set_pointcloud_color(color);
            }
        });
    }

    QWidget* gui() noexcept { return container; }

    auto set_pointcloud_color(const QColor& color) -> void { }
};

SideToolbar::SideToolbar(ThemeManager& manager)
    : pimpl { std::make_unique<Impl>(manager) } { }

SideToolbar::~SideToolbar() = default;

auto SideToolbar::gui() -> QWidget* const { return pimpl->gui(); }
