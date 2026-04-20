#include "png-map-edit.hh"

namespace pcs::event {

auto ApplyPngMapEdit::exec() noexcept -> Result {
    if (width == 0 || height == 0) {
        return std::unexpected { "PNG 地图尺寸无效" };
    }

    if (pixels.size() != width * height) {
        return std::unexpected { "PNG 像素缓冲尺寸不匹配" };
    }

    auto output      = pixels;
    const auto value = operation == PngMapEditOperation::Erase ? std::uint8_t { 255 }
                                                                : std::uint8_t { 0 };

    draw_line_with_thickness(output, width, height, from, to, thickness, value);

    return output;
}

auto ApplyPngMapEdit::redo() noexcept -> Result { return exec(); }

}
