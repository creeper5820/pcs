#include "config-option.hh"

namespace pcs {

ConfigOption::ConfigOption() noexcept
    : AppOption {
        QStringList {} << "c" << "config",
        "The path of config file",
        "config-path",
    } { }

auto ConfigOption::exec(Context& context) const -> std::expected<void, std::string> {
    if (!context.parser.isSet(*this)) {
        return { };
    }

    context.app.set_configuration_path(context.parser.value(*this).toStdString());
    return { };
}

}
