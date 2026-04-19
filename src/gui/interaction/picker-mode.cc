#include "gui/interaction/picker-mode.hh"

namespace pcs::gui::interaction {

PickerMode::PickerMode(Renderer& renderer, AssetsManager& assets) noexcept
    : renderer { renderer }
    , assets { assets } { }

auto PickerMode::register_extension(std::unique_ptr<PickerExtension> extension) noexcept -> void {
    if (extension == nullptr) {
        return;
    }

    extensions[extension->kind()] = std::move(extension);
}

auto PickerMode::id() const noexcept -> MouseModeId { return MouseModeId::Picker; }

auto PickerMode::on_init(Mouse& mouse) noexcept -> void {
    next_move_pick_at = std::chrono::steady_clock::time_point::min();
    mouse.set_status("拾取模式：移动鼠标查看坐标");

    mouse.on_move([this, &mouse](MouseEvent const& event) {
        const auto now = std::chrono::steady_clock::now();
        if (now < next_move_pick_at) {
            return;
        }

        next_move_pick_at = now + kMovePickInterval;
        pick(mouse, event);
    });

    mouse.on_lclick([this, &mouse](MouseEvent const& event) { pick(mouse, event); });
}

auto PickerMode::on_exit(Mouse& mouse) noexcept -> void { mouse.set_status("拾取模式：已关闭"); }

auto PickerMode::pick(Mouse& mouse, MouseEvent const& event) noexcept -> void {
    const auto& selected = mouse.selected_asset();
    if (!selected.has_value()) {
        mouse.set_status("拾取模式：未选择资产");
        return;
    }

    auto extension = extensions.find(selected->kind);
    if (extension == extensions.end() || extension->second == nullptr) {
        return;
    }

    extension->second->pick(mouse, event, selected->id, renderer, assets);
}

}
