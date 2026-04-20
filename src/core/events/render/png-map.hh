#pragma once

#include "core/handle/png-map.hh"

#include <expected>
#include <string>
#include <string_view>

namespace pcs::event {

struct MakePngMapUnit {
    using Result = std::expected<std::unique_ptr<PngMapHandle>, std::string>;

    struct Meta {
        std::string_view name = "Make PNG Map Unit";
        bool recordable       = false;
        bool redoable         = true;
    };

    Meta meta { };
    std::string path;
    std::string name;

    auto exec() noexcept -> Result;
    auto redo() noexcept -> Result;
};

}
