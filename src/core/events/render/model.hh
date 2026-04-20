#pragma once

#include "core/handle/model.hh"

#include <expected>
#include <string>
#include <string_view>

namespace pcs::event {

struct MakeModelUnit {
    using Result = std::expected<std::unique_ptr<ModelHandle>, std::string>;

    struct Meta {
        std::string_view name = "Make Model Unit";
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
