#pragma once

#include <cstddef>

namespace pcs {

struct ModelToPointcloudParameters {
    double density               = 1.0;
    double sample_distance       = 0.0;
    double unit_scale            = 1.0;
    bool include_edge_points     = true;
    bool include_interior_points = true;
    bool include_vertex_points   = false;
    std::size_t max_points       = 0;
};

}
