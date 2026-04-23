#include "gui/interaction/png-origin-pick-mode.hh"

#include "core/handle/png-map.hh"
#include "core/map/png-map-transform.hh"

#include <array>

namespace pcs::gui::interaction {

namespace {

    auto format_xyz(double const* p) noexcept -> QString {
        return QString("x=%1, y=%2, z=%3")
            .arg(p[0], 0, 'f', 4)
            .arg(p[1], 0, 'f', 4)
            .arg(p[2], 0, 'f', 4);
    }

    auto mode_status(QString const& detail = { }) noexcept -> QString {
        auto text = QString::fromUtf8("设置坐标系原点：点击 PNG 地图选择原点，右键取消");
        if (!detail.isEmpty()) {
            text += " | ";
            text += detail;
        }
        return text;
    }

}

PngOriginPickMode::PngOriginPickMode(Renderer& renderer, AssetsManager& assets) noexcept
    : renderer { renderer }
    , assets { assets } { }

auto PngOriginPickMode::id() const noexcept -> MouseModeId { return MouseModeId::PngOriginPick; }

auto PngOriginPickMode::on_init(Mouse& mouse) noexcept -> void {
    mouse.set_status(mode_status());
    mouse.on_move([this, &mouse](MouseEvent const& event) { on_move(mouse, event); });
    mouse.on_lclick([this, &mouse](MouseEvent const& event) { on_lclick(mouse, event); });
    mouse.on_rclick([this, &mouse](MouseEvent const&) { on_rclick(mouse); });
}

auto PngOriginPickMode::on_exit(Mouse&) noexcept -> void { }

auto PngOriginPickMode::supports_selection(
    std::optional<MouseSelection> const& selection) const noexcept -> bool {
    return selection.has_value() && selection->kind == PngMapHandle::kKind;
}

auto PngOriginPickMode::allows_camera_interaction(Mouse const&) const noexcept -> bool {
    return false;
}

auto PngOriginPickMode::on_move(Mouse& mouse, MouseEvent const& event) noexcept -> void {
    const auto* request = mouse.png_origin_pick_request();
    if (request == nullptr) {
        mouse.set_mode(MouseModeId::PngEdit);
        return;
    }

    auto handle = assets.get_handle<PngMapHandle>(request->asset_id);
    if (!handle.has_value() || handle.value() == nullptr) {
        mouse.set_status(mode_status(QString::fromUtf8("PNG 地图不可用")));
        return;
    }

    auto position = handle.value()->pick_plane_position(renderer, event.x, event.y);
    if (!position.has_value()) {
        mouse.set_status(mode_status(QString::fromUtf8("未命中地图平面")));
        return;
    }

    auto pixel = png_map_pixel_from_world(handle.value()->transform_view(),
        std::array<double, 3> {
            std::get<0>(*position), std::get<1>(*position), std::get<2>(*position) });
    if (!pixel.has_value()) {
        mouse.set_status(mode_status(QString::fromUtf8("未命中有效像素")));
        return;
    }

    const auto frame = png_map_frame_from_pixel(handle.value()->transform_view(), *pixel);
    mouse.set_status(mode_status(QString::fromUtf8("预览 %1").arg(format_xyz(frame.data()))));
}

auto PngOriginPickMode::on_lclick(Mouse& mouse, MouseEvent const& event) noexcept -> void {
    const auto* request = mouse.png_origin_pick_request();
    if (request == nullptr) {
        mouse.set_mode(MouseModeId::PngEdit);
        return;
    }

    auto handle = assets.get_handle<PngMapHandle>(request->asset_id);
    if (!handle.has_value() || handle.value() == nullptr) {
        mouse.set_status(mode_status(QString::fromUtf8("PNG 地图不可用")));
        return;
    }

    auto position = handle.value()->pick_plane_position(renderer, event.x, event.y);
    if (!position.has_value()) {
        mouse.set_status(mode_status(QString::fromUtf8("未命中地图平面")));
        return;
    }

    auto pixel = png_map_pixel_from_world(handle.value()->transform_view(),
        std::array<double, 3> {
            std::get<0>(*position), std::get<1>(*position), std::get<2>(*position) });
    if (!pixel.has_value()) {
        mouse.set_status(mode_status(QString::fromUtf8("未命中有效像素")));
        return;
    }

    auto callback = request->on_pick;
    mouse.cancel_png_origin_pick();
    mouse.set_mode(MouseModeId::PngEdit);
    if (callback) {
        callback(*pixel);
    }
}

auto PngOriginPickMode::on_rclick(Mouse& mouse) noexcept -> void {
    const auto* request = mouse.png_origin_pick_request();
    auto callback       = request != nullptr ? request->on_cancel : std::function<void()> { };
    mouse.cancel_png_origin_pick();
    mouse.set_mode(MouseModeId::PngEdit);
    if (callback) {
        callback();
    }
}

}
