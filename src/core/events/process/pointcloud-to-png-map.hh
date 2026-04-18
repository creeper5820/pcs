#pragma once

#include "core/events/common.hh"
#include "core/map/png-map-data.hh"

#include <expected>
#include <tuple>
#include <vector>

namespace pcs::event {

struct ConvertPointcloudToPngMap {
    using Position = std::tuple<double, double, double>;
    using Result   = std::expected<PngMapData, std::string>;

    struct Context {
        static constexpr EventMeta meta {
            .name      = "Convert Pointcloud To PNG Map",
            .consuming = true,
        };

        std::vector<Position> points;
        PngMapParameters parameters;
    };

    static auto runtime_exec(std::unique_ptr<Context>) noexcept -> Result;
};

}
