#include "png-map-edit.hh"

namespace pcs::event {

auto ApplyPngMapEdit::runtime_exec(std::unique_ptr<Context> context) noexcept -> Result {
    if (context == nullptr) {
        return std::unexpected { "PNG 编辑上下文为空" };
    }

    if (context->width == 0 || context->height == 0) {
        return std::unexpected { "PNG 地图尺寸无效" };
    }

    if (context->pixels.size() != context->width * context->height) {
        return std::unexpected { "PNG 像素缓冲尺寸不匹配" };
    }

    auto pixels = std::move(context->pixels);
    const auto value =
        context->operation == PngMapEditOperation::Erase ? std::uint8_t { 255 } : std::uint8_t { 0 };

    draw_line_with_thickness(pixels, context->width, context->height, context->from, context->to,
        context->thickness, value);

    return pixels;
}

}
