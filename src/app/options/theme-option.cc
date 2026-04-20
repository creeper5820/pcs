#include "theme-option.hh"

#include <spdlog/spdlog.h>

namespace pcs {

ThemeOption::ThemeOption() noexcept
    : AppOption {
        "theme",
        "Set the application's color theme, e.g., 'green' or 'dark'.",
        "theme-name",
    } { }

auto ThemeOption::exec(Context& context) const -> std::expected<void, std::string> {
    if (!context.parser.isSet(*this)) {
        return { };
    }

    context.app.set_theme_name(context.parser.value(*this).toStdString());
    return { };
}

ListThemesOption::ListThemesOption() noexcept
    : AppOption {
        "list-themes",
        "List all available theme colors.",
    } { }

auto ListThemesOption::exec(Context& context) const -> std::expected<void, std::string> {
    if (!context.parser.isSet(*this)) {
        return { };
    }

    spdlog::info("Available themes: green, blue-miku");
    context.app.request_exit();
    return { };
}

}
