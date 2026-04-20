#pragma once

#include "app-option.hh"

namespace pcs {

class FileOption final : public AppOption {
public:
    FileOption() noexcept;

    auto exec(Context& context) const -> std::expected<void, std::string> override;
};

}
