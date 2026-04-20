#include "egg-option.hh"

#include <spdlog/spdlog.h>

namespace pcs {

EggOption::EggOption() noexcept
    : AppOption {
        "show-egg",
        "Show a egg tip",
    } { }

auto EggOption::exec(Context& context) const -> std::expected<void, std::string> {
    if (!context.parser.isSet(*this)) {
        return { };
    }

    spdlog::info("Egg triggered! 🥚");
    context.app.request_exit();
    return { };
}

}
