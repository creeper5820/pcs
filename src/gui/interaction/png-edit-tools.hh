#pragma once

#include "gui/interaction/mouse.hh"

#include <vector>

#include <QString>

namespace pcs::gui::interaction {

struct PngEditToolDescriptor {
    PngEditTool id;
    QString label;
    QString icon;
    int param_index                = 0;
    bool allows_camera_interaction = false;
};

auto default_png_edit_tool_descriptors() noexcept -> std::vector<PngEditToolDescriptor> const&;
auto png_edit_tool_descriptor(PngEditTool tool) noexcept -> PngEditToolDescriptor const&;

}
