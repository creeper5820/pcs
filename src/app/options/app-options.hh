#pragma once

#include "app-option.hh"

#include <creeper-qt/core/application.hh>

#include <memory>
#include <utility>
#include <vector>

namespace pcs {

class AppOptions {
public:
    AppOptions() noexcept;

    template <typename... Options>
    explicit AppOptions(Options&&... options) noexcept
        : AppOptions {} {
        (add(std::forward<Options>(options)), ...);
    }

    auto add(std::unique_ptr<AppOption>) noexcept -> void;
    auto process(QCoreApplication const& application) noexcept -> void;
    auto exec(App& app) const -> std::expected<void, std::string>;

private:
    QCommandLineParser parser;
    std::vector<std::unique_ptr<AppOption>> options;
};

}
