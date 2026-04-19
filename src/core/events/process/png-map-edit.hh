#pragma once

#include "core/events/common.hh"
#include "core/map/png-edit-ops.hh"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <vector>

namespace pcs::event {

enum class PngMapEditOperation {
    Draw,
    Erase,
};

struct ApplyPngMapEdit {
    using Result = std::expected<std::vector<std::uint8_t>, std::string>;

    struct Context {
        static constexpr EventMeta meta {
            .name      = "Apply PNG Map Edit",
            .consuming = true,
        };

        std::vector<std::uint8_t> pixels;
        std::size_t width  = 0;
        std::size_t height = 0;
        PixelPoint from;
        PixelPoint to;
        std::size_t thickness = 1;
        PngMapEditOperation operation { PngMapEditOperation::Draw };
    };

    static auto runtime_exec(std::unique_ptr<Context>) noexcept -> Result;
};

}
