#include "app-options.hh"

namespace pcs {

AppOptions::AppOptions() noexcept {
    parser.setApplicationDescription(parser.tr("Point Cloud Shop With PCL Backend"));
    parser.addVersionOption();
    parser.addHelpOption();
}

auto AppOptions::add(std::unique_ptr<AppOption> option) noexcept -> void {
    if (option == nullptr) {
        return;
    }

    parser.addOption(*option);
    options.push_back(std::move(option));
}

auto AppOptions::process(QCoreApplication const& application) noexcept -> void {
    parser.process(application);
}

auto AppOptions::exec(App& app) const -> std::expected<void, std::string> {
    auto context = AppOption::Context {
        .parser = parser,
        .app    = app,
    };

    for (auto const& option : options) {
        if (option == nullptr) {
            continue;
        }

        if (auto result = option->exec(context); !result.has_value()) {
            return result;
        }
    }

    return { };
}

}
