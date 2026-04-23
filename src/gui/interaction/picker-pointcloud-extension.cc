#include "gui/interaction/picker-pointcloud-extension.hh"

#include "core/handle/points.hh"

#include <array>

namespace pcs::gui::interaction {

namespace {

    auto format_xyz(double const* p) noexcept -> QString {
        return QString("x=%1, y=%2, z=%3")
            .arg(p[0], 0, 'f', 4)
            .arg(p[1], 0, 'f', 4)
            .arg(p[2], 0, 'f', 4);
    }

    class PointcloudPickerExtension final : public PickerExtension {
    public:
        auto kind() const noexcept -> std::string_view override { return PointsHandle::kKind; }

        auto pick(Mouse& mouse, MouseEvent const& event, std::string const& asset_id,
            Renderer& renderer, AssetsManager& assets) noexcept -> void override {
            auto handle = assets.get_handle<PointsHandle>(asset_id);
            if (!handle.has_value() || handle.value() == nullptr) {
                mouse.set_status("拾取模式：点云不可用");
                return;
            }

            auto position = handle.value()->pick_position(renderer, event.x, event.y);
            if (!position.has_value()) {
                mouse.set_status("拾取模式：未命中点云点");
                return;
            }

            const auto [x, y, z] = *position;
            const auto value     = std::array<double, 3> { x, y, z };
            mouse.set_status(QString("点 | %1").arg(format_xyz(value.data())));
        }
    };

}

auto make_pointcloud_picker_extension() -> std::unique_ptr<PickerExtension> {
    return std::make_unique<PointcloudPickerExtension>();
}

}
