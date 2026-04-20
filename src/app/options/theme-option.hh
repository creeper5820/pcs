#pragma once

#include "app-option.hh"

namespace pcs {

class ThemeOption final : public AppOption {
public:
    ThemeOption() noexcept;

    auto exec(Context& context) const -> std::expected<void, std::string> override;
};

class ListThemesOption final : public AppOption {
public:
    ListThemesOption() noexcept;

    auto exec(Context& context) const -> std::expected<void, std::string> override;
};

}
