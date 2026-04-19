#pragma once

#include "gui/interaction/mouse.hh"

#include <creeper-qt/utility/theme/theme.hh>

#include <qpointer.h>
#include <qwidget.h>

struct NavigationState {
    creeper::ThemeManager& manager;

    std::string icon_font;
    pcs::gui::interaction::Mouse* mouse = nullptr;

    struct ButtonContext {
        std::string_view name;
        std::string_view icon;
        std::function<void()> callback;
    };
    std::vector<ButtonContext> buttons_context;

    std::function<void(bool)> picker_mode_setter;
    std::function<bool()> picker_mode_getter;

    std::function<void()> function_quit;
};
auto NavigationComponent(NavigationState& state) noexcept -> QPointer<QWidget>;
