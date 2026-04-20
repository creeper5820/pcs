#pragma once

#include "core/handle/points.hh"

#include <expected>
#include <string>
#include <string_view>

namespace pcs::event {

struct MakePointsUnit {
    using Result = std::expected<std::unique_ptr<PointsHandle>, std::string>;

    struct Meta {
        std::string_view name = "Make Points Unit";
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
