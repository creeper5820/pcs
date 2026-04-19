#pragma once

#include "gui/interaction/picker-extension.hh"

#include <memory>

namespace pcs::gui::interaction {

auto make_png_map_picker_extension() -> std::unique_ptr<PickerExtension>;

}
