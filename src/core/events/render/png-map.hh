#pragma once

#include "core/events/common.hh"
#include "core/handle/png-map.hh"

#include <expected>

namespace pcs::event {

struct MakePngMapUnit {
    using Result = std::expected<std::unique_ptr<PngMapHandle>, std::string>;

    struct Context {
        static constexpr EventMeta meta {
            .name      = "Make PNG Map Unit",
            .consuming = true,
        };
        std::string path;
        std::string name;
    };

    static auto runtime_exec(std::unique_ptr<Context>) noexcept -> Result;
};

}
