#pragma once
#include "utility/pimpl.hh"

#include <string>
#include <vector>

namespace pcs {

class App final {
    PCS_PIMPL_DEFINITION(App)

public:
    auto show() noexcept -> void;

    auto set_configuration_path(std::string const&) noexcept -> void;
    auto set_startup_files(std::vector<std::string>) noexcept -> void;
    auto set_theme_name(std::string const&) noexcept -> void;
    auto request_exit() noexcept -> void;
    auto should_exit() const noexcept -> bool;
};

}
