#pragma once

#include <cstddef>

namespace pcs {

struct ModelToPointcloudParameters {
    double density         = 10.0;
    double sample_distance = 0.0;
    double unit_scale      = 1.0;
    std::size_t max_points = 0;
};

}
