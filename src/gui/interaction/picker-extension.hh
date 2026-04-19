#pragma once

#include "core/assets.hh"
#include "core/renderer.hh"
#include "gui/interaction/mouse.hh"

#include <string>

namespace pcs::gui::interaction {

struct PickerExtension {
    virtual ~PickerExtension() = default;

    virtual auto kind() const noexcept -> pcs::AssetKind = 0;

    virtual auto pick(Mouse&, MouseEvent const&, std::string const& asset_id, Renderer&,
        AssetsManager&) noexcept -> void = 0;
};

}
