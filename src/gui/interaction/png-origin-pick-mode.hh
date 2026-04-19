#pragma once

#include "core/assets.hh"
#include "core/renderer.hh"
#include "gui/interaction/mouse.hh"

namespace pcs::gui::interaction {

class PngOriginPickMode final : public MouseMode {
public:
    PngOriginPickMode(Renderer&, AssetsManager&) noexcept;

    auto id() const noexcept -> MouseModeId override;
    auto on_init(Mouse&) noexcept -> void override;
    auto on_exit(Mouse&) noexcept -> void override;
    auto supports_selection(std::optional<MouseSelection> const&) const noexcept -> bool override;
    auto allows_camera_interaction(Mouse const&) const noexcept -> bool override;

private:
    auto on_move(Mouse&, MouseEvent const&) noexcept -> void;
    auto on_lclick(Mouse&, MouseEvent const&) noexcept -> void;
    auto on_rclick(Mouse&) noexcept -> void;

    Renderer& renderer;
    AssetsManager& assets;
};

}
