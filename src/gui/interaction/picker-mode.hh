#pragma once

#include "core/assets.hh"
#include "core/renderer.hh"
#include "gui/interaction/mouse.hh"
#include "gui/interaction/picker-extension.hh"

#include <chrono>
#include <memory>
#include <unordered_map>

namespace pcs::gui::interaction {

class PickerMode final : public MouseMode {
public:
    PickerMode(Renderer&, AssetsManager&) noexcept;

    auto register_extension(std::unique_ptr<PickerExtension>) noexcept -> void;

    auto id() const noexcept -> MouseModeId override;
    auto on_init(Mouse&) noexcept -> void override;
    auto on_exit(Mouse&) noexcept -> void override;

private:
    auto pick(Mouse&, MouseEvent const&) noexcept -> void;

    Renderer& renderer;
    AssetsManager& assets;
    std::unordered_map<pcs::AssetKind, std::unique_ptr<PickerExtension>> extensions;

    std::chrono::steady_clock::time_point next_move_pick_at;
    static constexpr auto kMovePickInterval = std::chrono::milliseconds { 100 };
};

}
