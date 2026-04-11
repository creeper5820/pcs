#pragma once
#include "utility/pimpl.hh"

namespace pcs {

class App final {
    PCS_PIMPL_DEFINITION(App)

public:
    auto show() noexcept -> void;

    auto set_configuration_path(std::string const&) noexcept -> void;
};

}
