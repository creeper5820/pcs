#include "gui/interaction/picker-png-map-extension.hh"

#include "core/handle/png-map.hh"

#include <array>

namespace pcs::gui::interaction {

namespace {

    auto format_xyz(double const* p) noexcept -> QString {
        return QString("x=%1, y=%2, z=%3")
            .arg(p[0], 0, 'f', 4)
            .arg(p[1], 0, 'f', 4)
            .arg(p[2], 0, 'f', 4);
    }

    class PngMapPickerExtension final : public PickerExtension {
    public:
        auto kind() const noexcept -> std::string_view override { return PngMapHandle::kKind; }

        auto pick(Mouse& mouse, MouseEvent const& event, std::string const& asset_id,
            Renderer& renderer, AssetsManager& assets) noexcept -> void override {
            auto handle = assets.get_handle<PngMapHandle>(asset_id);
            if (!handle.has_value() || handle.value() == nullptr) {
                mouse.set_status("拾取模式：PNG 地图不可用");
                return;
            }

            auto position = handle.value()->pick_plane_position(renderer, event.x, event.y);
            if (!position.has_value()) {
                mouse.set_status("拾取模式：未命中地图平面");
                return;
            }

            const auto [x, y, z] = *position;
            const auto value     = std::array<double, 3> { x, y, z };
            mouse.set_status(QString("平面 | %1").arg(format_xyz(value.data())));
        }
    };

}

auto make_png_map_picker_extension() -> std::unique_ptr<PickerExtension> {
    return std::make_unique<PngMapPickerExtension>();
}

}
