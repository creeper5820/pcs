#include "gui/interaction/png-edit-tools.hh"

#include <array>

namespace pcs::gui::interaction {

auto default_png_edit_tool_descriptors() noexcept -> std::vector<PngEditToolDescriptor> const& {
    static const auto descriptors = std::vector<PngEditToolDescriptor> {
        { PngEditTool::Free, "自由", "mouse", 0, true },
        { PngEditTool::Line, "线", "timeline", 1, false },
        { PngEditTool::Point, "点", "fiber_manual_record", 2, false },
        { PngEditTool::Erase, "擦除", "delete", 3, false },
    };

    return descriptors;
}

auto png_edit_tool_descriptor(PngEditTool tool) noexcept -> PngEditToolDescriptor const& {
    for (auto const& descriptor : default_png_edit_tool_descriptors()) {
        if (descriptor.id == tool) {
            return descriptor;
        }
    }

    return default_png_edit_tool_descriptors().front();
}

}
