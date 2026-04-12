#pragma once

#include "core/events/common.hh"
#include "core/handle/model.hh"

#include <expected>

namespace pcs::event {

struct MakeModelUnit {
    using Result = std::expected<std::unique_ptr<ModelHandle>, std::string>;

    struct Context {
        static constexpr EventMeta meta {
            .name      = "Make Model Unit",
            .consuming = true,
        };
        std::string path;
        std::string name;
    };

    static auto runtime_exec(std::unique_ptr<Context>) noexcept -> Result;
};

}
