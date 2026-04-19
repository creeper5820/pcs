#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace pcs {

enum class PngMapExportMirror {
    Horizontal,
    Vertical,
    None,
};

struct PngMapFrameConfig {
    double yaw_deg = 0.0;
    std::optional<int> origin_pixel_x;
    std::optional<int> origin_pixel_y;
    PngMapExportMirror export_mirror { PngMapExportMirror::Horizontal };
};

struct PngMapParameters {
    double resolution        = 0.1;
    std::size_t points_limit = 5;
    double height_limit      = 0.1;
    double influence_radius  = 0.08;
    double z_area_start      = 0.0;
    double z_area_end        = 1.0;
};

struct PngMapData {
    std::vector<std::uint8_t> pixels;

    std::size_t width  = 0;
    std::size_t height = 0;

    double origin_x   = 0.0;
    double origin_y   = 0.0;
    double resolution = 0.1;

    double plane_z = -0.02;

    double z_area_start = 0.0;
    double z_area_end   = 1.0;

    PngMapFrameConfig frame_config { };
};

}
