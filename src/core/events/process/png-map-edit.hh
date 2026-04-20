#pragma once

#include "core/map/png-edit-ops.hh"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <string>
#include <string_view>
#include <vector>

namespace pcs::event {

enum class PngMapEditOperation {
    Draw,
    Erase,
};

struct ApplyPngMapEdit {
    using Result = std::expected<std::vector<std::uint8_t>, std::string>;

    struct Meta {
        std::string_view name = "Apply PNG Map Edit";
        bool recordable       = false;
        bool redoable         = true;
    };

    Meta meta { };
    std::vector<std::uint8_t> pixels;
    std::size_t width  = 0;
    std::size_t height = 0;
    PixelPoint from;
    PixelPoint to;
    std::size_t thickness = 1;
    PngMapEditOperation operation { PngMapEditOperation::Draw };

    auto exec() noexcept -> Result;
    auto redo() noexcept -> Result;
};

}
