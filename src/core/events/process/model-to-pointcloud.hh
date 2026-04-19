#pragma once

#include "core/events/common.hh"
#include "core/map/model-data.hh"
#include "core/map/model-pointcloud-data.hh"

#include <expected>
#include <tuple>
#include <vector>

namespace pcs::event {

struct ConvertModelToPointcloud {
    using Position = std::tuple<double, double, double>;
    using Result   = std::expected<std::vector<Position>, std::string>;

    struct Context {
        static constexpr EventMeta meta {
            .name      = "Convert Model To Pointcloud",
            .consuming = true,
        };

        ModelData model;
        ModelToPointcloudParameters parameters;
    };

    static auto runtime_exec(std::unique_ptr<Context>) noexcept -> Result;
};

}
