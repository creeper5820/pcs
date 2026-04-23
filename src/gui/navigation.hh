#pragma once

#include "gui/interaction/mouse.hh"

#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/utility/wrapper/pimpl.hh>

#include <QWidget>

#include <functional>
#include <string>
#include <string_view>
#include <vector>

class Navigation final : public QWidget {
    CREEPER_PIMPL_DEFINITION(Navigation)

public:
    struct ButtonContext {
        std::string_view name;
        std::string_view icon;
        std::function<void()> callback;
    };

    Navigation(creeper::ThemeManager& manager, std::string icon_font,
        std::vector<ButtonContext> buttons_context, std::function<void(bool)> picker_mode_setter,
        std::function<bool()> picker_mode_getter, std::function<void()> function_quit,
        pcs::gui::interaction::Mouse& mouse) noexcept;
};
