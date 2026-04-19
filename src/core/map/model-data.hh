#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace pcs {

struct ModelData {
    std::vector<std::array<double, 3>> vertices;
    std::vector<std::array<std::size_t, 3>> faces;
};

}
