#pragma once

#include "app-option.hh"

namespace pcs {

class EggOption final : public AppOption {
public:
    EggOption() noexcept;

    auto exec(Context& context) const -> std::expected<void, std::string> override;
};

}
