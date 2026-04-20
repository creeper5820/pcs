#pragma once

#include "core/map/model-data.hh"
#include "core/map/model-pointcloud-data.hh"

#include <expected>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace pcs::event {

struct ConvertModelToPointcloud {
    using Position = std::tuple<double, double, double>;
    using Result   = std::expected<std::vector<Position>, std::string>;

    struct Meta {
        std::string_view name = "Convert Model To Pointcloud";
        bool recordable       = false;
        bool redoable         = true;
    };

    Meta meta { };
    ModelData model;
    ModelToPointcloudParameters parameters;

    auto exec() noexcept -> Result;
    auto redo() noexcept -> Result;
};

}
