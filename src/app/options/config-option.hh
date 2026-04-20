#pragma once

#include "app-option.hh"

namespace pcs {

class ConfigOption final : public AppOption {
public:
    ConfigOption() noexcept;

    auto exec(Context& context) const -> std::expected<void, std::string> override;
};

}
