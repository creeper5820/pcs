#pragma once

#include <expected>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include <QString>

namespace pcs::gui::working {

class OpenControl {
public:
    struct Format {
        std::string label;
        std::string dialog_pattern;
        std::vector<std::string> extensions;
        std::function<std::expected<void, std::string>(std::string const& path)> open;
    };

    auto register_format(Format) noexcept -> void;

    auto dialog_filter() const noexcept -> QString;
    auto open(std::string const& path) const noexcept -> std::expected<void, std::string>;

private:
    auto find_format(std::string_view extension) const noexcept -> Format const*;

    std::vector<Format> formats;
};

}
