#pragma once

#include "core/events/common.hh"
#include "core/handle/points.hh"

#include <expected>

namespace pcs::event {

struct MakePointsUnit {
    using Result = std::expected<std::unique_ptr<PointsHandle>, std::string>;

    struct Context {
        static constexpr EventMeta meta {
            .name      = "Make Points Unit",
            .consuming = true,
        };
        std::string path;
        std::string name;
    };

    static auto runtime_exec(std::unique_ptr<Context>) noexcept -> Result;
};

}
