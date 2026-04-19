#pragma once

#include "gui/interaction/picker-extension.hh"

#include <memory>

namespace pcs::gui::interaction {

auto make_pointcloud_picker_extension() -> std::unique_ptr<PickerExtension>;

}
