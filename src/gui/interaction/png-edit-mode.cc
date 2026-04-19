#include "gui/interaction/png-edit-mode.hh"

#include "core/handle/png-map.hh"
#include "core/map/png-map-transform.hh"
#include "gui/interaction/png-edit-tools.hh"

#include <algorithm>
#include <array>
#include <cmath>

namespace pcs::gui::interaction {

namespace {

    auto format_xyz(double const* p) noexcept -> QString {
        return QString("x=%1, y=%2, z=%3")
            .arg(p[0], 0, 'f', 4)
            .arg(p[1], 0, 'f', 4)
            .arg(p[2], 0, 'f', 4);
    }

    auto mode_status(Mouse const& mouse, QString const& detail = { }) noexcept -> QString {
        auto text = QString("PNG 编辑 | 模式: %1")
                        .arg(png_edit_tool_descriptor(mouse.png_edit_tool()).label);
        if (!detail.isEmpty()) {
            text += " | ";
            text += detail;
        }
        return text;
    }

    auto to_pixel(PngMapHandle const& handle, PngMapHandle::Position const& world) noexcept
        -> std::optional<PixelPoint> {
        const auto [wx, wy, _] = world;

        return png_map_pixel_from_world(
            handle.transform_view(), std::array<double, 3> { wx, wy, handle.get_plane_z() });
    }

    auto from_pixel(PngMapHandle const& handle, PixelPoint point) noexcept
        -> std::array<double, 3> {
        return png_map_frame_from_pixel(handle.transform_view(), point);
    }

}

PngEditMode::PngEditMode(Renderer& renderer, AssetsManager& assets) noexcept
    : renderer { renderer }
    , assets { assets } {
    register_tool_handler(PngEditTool::Free,
        ToolHandler {
            .on_move = [](auto& self, Mouse& mouse, MouseEvent const&, EditContext& context) {
                context.session.last_drag_point.reset();
                self.clear_preview(context.handle, context.session);
                if (!context.pixel.has_value()) {
                    return;
                }

                const auto world = from_pixel(context.handle, *context.pixel);
                mouse.set_status(mode_status(mouse, QString("坐标 %1").arg(format_xyz(world.data()))));
            },
            .on_lclick = [](auto& self, Mouse& mouse, MouseEvent const&, EditContext& context) {
                self.clear_preview(context.handle, context.session);
                if (!context.pixel.has_value()) {
                    mouse.set_status(mode_status(mouse, "未命中平面"));
                    return;
                }

                const auto world = from_pixel(context.handle, *context.pixel);
                mouse.set_status(mode_status(mouse, QString("坐标 %1").arg(format_xyz(world.data()))));
            },
        });
    register_tool_handler(PngEditTool::Point,
        ToolHandler {
            .on_move = [](auto& self, Mouse& mouse, MouseEvent const& event, EditContext& context) {
                self.clear_preview(context.handle, context.session);
                if (!context.pixel.has_value()) {
                    context.session.last_drag_point.reset();
                    return;
                }
                if ((event.buttons & Qt::LeftButton) == 0) {
                    context.session.last_drag_point.reset();
                    return;
                }

                const auto from = context.session.last_drag_point.value_or(*context.pixel);
                if (!self.apply_stroke(mouse, context.handle, context.session,
                        event::PngMapEditOperation::Draw, from, *context.pixel,
                        mouse.png_edit_point_size(), "连续绘制")) {
                    return;
                }

                context.session.last_drag_point = *context.pixel;
            },
            .on_lclick = [](auto& self, Mouse& mouse, MouseEvent const&, EditContext& context) {
                if (!context.pixel.has_value()) {
                    mouse.set_status(mode_status(mouse, "未命中平面"));
                    return;
                }
                context.session.last_drag_point.reset();
                self.clear_preview(context.handle, context.session);
                self.apply_stroke(mouse, context.handle, context.session,
                    event::PngMapEditOperation::Draw, *context.pixel, *context.pixel,
                    mouse.png_edit_point_size(), "绘制点");
            },
        });
    register_tool_handler(PngEditTool::Erase,
        ToolHandler {
            .on_move = [](auto& self, Mouse& mouse, MouseEvent const& event, EditContext& context) {
                self.clear_preview(context.handle, context.session);
                if (!context.pixel.has_value()) {
                    context.session.last_drag_point.reset();
                    return;
                }
                if ((event.buttons & Qt::LeftButton) == 0) {
                    context.session.last_drag_point.reset();
                    return;
                }

                const auto from = context.session.last_drag_point.value_or(*context.pixel);
                if (!self.apply_stroke(mouse, context.handle, context.session,
                        event::PngMapEditOperation::Erase, from, *context.pixel,
                        mouse.png_edit_erase_size(), "连续擦除")) {
                    return;
                }

                context.session.last_drag_point = *context.pixel;
            },
            .on_lclick = [](auto& self, Mouse& mouse, MouseEvent const&, EditContext& context) {
                if (!context.pixel.has_value()) {
                    mouse.set_status(mode_status(mouse, "未命中平面"));
                    return;
                }
                context.session.last_drag_point.reset();
                self.clear_preview(context.handle, context.session);
                self.apply_stroke(mouse, context.handle, context.session,
                    event::PngMapEditOperation::Erase, *context.pixel, *context.pixel,
                    mouse.png_edit_erase_size(), "擦除");
            },
        });
    register_tool_handler(PngEditTool::Line,
        ToolHandler {
            .on_move = [](auto& self, Mouse& mouse, MouseEvent const&, EditContext& context) {
                context.session.last_drag_point.reset();
                if (!context.session.pending_line_start.has_value() || !context.pixel.has_value()) {
                    self.clear_preview(context.handle, context.session);
                    return;
                }

                if (!self.show_preview(context.handle, context.session, *context.pixel,
                        mouse.png_edit_line_width())) {
                    mouse.set_status(mode_status(mouse, "线段预览失败"));
                    return;
                }

                const auto world = from_pixel(context.handle, *context.pixel);
                mouse.set_status(
                    mode_status(mouse, QString("预览终点 %1").arg(format_xyz(world.data()))));
            },
            .on_lclick = [](auto& self, Mouse& mouse, MouseEvent const&, EditContext& context) {
                if (!context.pixel.has_value()) {
                    mouse.set_status(mode_status(mouse, "未命中平面"));
                    return;
                }

                context.session.last_drag_point.reset();
                if (!context.session.pending_line_start.has_value()) {
                    context.session.pending_line_start = *context.pixel;
                    const auto world = from_pixel(context.handle, *context.pixel);
                    mouse.set_status(
                        mode_status(mouse, QString("线起点 %1").arg(format_xyz(world.data()))));
                    return;
                }

                auto result = self.apply_edit(context.session, context.handle,
                    event::PngMapEditOperation::Draw, *context.session.pending_line_start,
                    *context.pixel, mouse.png_edit_line_width());
                if (!result.has_value()) {
                    mouse.set_status(mode_status(mouse, "应用线绘制失败"));
                    return;
                }

                context.session.pixels = std::move(result.value());
                context.session.pending_line_start.reset();
                self.clear_preview(context.handle, context.session);
                if (!self.commit_session(context.handle, context.session)) {
                    mouse.set_status(mode_status(mouse, "应用线绘制失败"));
                    return;
                }

                const auto world = from_pixel(context.handle, *context.pixel);
                mouse.set_status(mode_status(
                    mouse, QString("绘制线段至 %1").arg(format_xyz(world.data()))));
            },
            .on_rclick = [](auto& self, Mouse& mouse, MouseEvent const&, EditContext& context) {
                context.session.last_drag_point.reset();
                if (!context.session.pending_line_start.has_value()) {
                    return;
                }

                context.session.pending_line_start.reset();
                self.clear_preview(context.handle, context.session);
                mouse.set_status(mode_status(mouse, "已取消线绘制"));
            },
        });
}

auto PngEditMode::id() const noexcept -> MouseModeId { return MouseModeId::PngEdit; }

auto PngEditMode::supports_selection(std::optional<MouseSelection> const& selection) const noexcept
    -> bool {
    return selection.has_value() && selection->kind == pcs::AssetKind::PngMap;
}

auto PngEditMode::allows_camera_interaction(Mouse const& mouse) const noexcept -> bool {
    return png_edit_tool_descriptor(mouse.png_edit_tool()).allows_camera_interaction;
}

auto PngEditMode::on_init(Mouse& mouse) noexcept -> void {
    mouse.set_status(mode_status(mouse));

    mouse.on_move([this, &mouse](MouseEvent const& event) { on_move(mouse, event); });
    mouse.on_lclick([this, &mouse](MouseEvent const& event) { on_lclick(mouse, event); });
    mouse.on_rclick([this, &mouse](MouseEvent const& event) { on_rclick(mouse, event); });
}

auto PngEditMode::on_exit(Mouse& mouse) noexcept -> void {
    for (auto& [asset_id, session] : sessions) {
        auto handle = assets.get_png_map_handle(asset_id);
        if (handle.has_value() && handle.value() != nullptr) {
            handle.value()->clear_preview();
        }
        session.preview_visible = false;
    }

    sessions.clear();

    mouse.set_status("PNG 编辑: 关闭");
}

auto PngEditMode::on_move(Mouse& mouse, MouseEvent const& event) noexcept -> void {
    auto context = resolve_edit_context(mouse, event);
    if (!context.has_value()) {
        return;
    }

    auto iter = tool_handlers.find(mouse.png_edit_tool());
    if (iter == tool_handlers.end() || !iter->second.on_move) {
        return;
    }

    iter->second.on_move(*this, mouse, event, *context);
}

auto PngEditMode::on_lclick(Mouse& mouse, MouseEvent const& event) noexcept -> void {
    auto context = resolve_edit_context(mouse, event);
    if (!context.has_value()) {
        return;
    }

    auto iter = tool_handlers.find(mouse.png_edit_tool());
    if (iter == tool_handlers.end() || !iter->second.on_lclick) {
        return;
    }

    iter->second.on_lclick(*this, mouse, event, *context);
}

auto PngEditMode::on_rclick(Mouse& mouse, MouseEvent const& event) noexcept -> void {
    auto context = resolve_edit_context(mouse, event);
    if (!context.has_value()) {
        return;
    }

    auto iter = tool_handlers.find(mouse.png_edit_tool());
    if (iter == tool_handlers.end() || !iter->second.on_rclick) {
        return;
    }

    iter->second.on_rclick(*this, mouse, event, *context);
}

auto PngEditMode::register_tool_handler(PngEditTool tool, ToolHandler handler) noexcept -> void {
    tool_handlers[tool] = std::move(handler);
}

auto PngEditMode::resolve_edit_context(Mouse& mouse, MouseEvent const& event) noexcept
    -> std::optional<EditContext> {
    const auto& selected = mouse.selected_asset();
    if (!selected.has_value() || selected->kind != pcs::AssetKind::PngMap) {
        return std::nullopt;
    }

    auto handle = assets.get_png_map_handle(selected->id);
    if (!handle.has_value() || handle.value() == nullptr) {
        mouse.set_status(mode_status(mouse, "PNG 地图不可用"));
        return std::nullopt;
    }

    auto* session = ensure_session(selected->id, *handle.value());
    if (session == nullptr) {
        mouse.set_status(mode_status(mouse, "会话初始化失败"));
        return std::nullopt;
    }

    return EditContext {
        .asset_id = selected->id,
        .handle   = *handle.value(),
        .session  = *session,
        .pixel    = pick_pixel(selected->id, event),
    };
}

auto PngEditMode::pick_pixel(std::string const& asset_id, MouseEvent const& event) const noexcept
    -> std::optional<PixelPoint> {
    auto handle = assets.get_png_map_handle(asset_id);
    if (!handle.has_value() || handle.value() == nullptr) {
        return std::nullopt;
    }

    auto world = handle.value()->pick_plane_position(renderer, event.x, event.y);
    if (!world.has_value()) {
        return std::nullopt;
    }

    return to_pixel(*handle.value(), *world);
}

auto PngEditMode::ensure_session(std::string const& asset_id, PngMapHandle& handle) noexcept
    -> Session* {
    auto [iter, inserted]    = sessions.try_emplace(asset_id);
    const auto expected_size = handle.get_width() * handle.get_height();
    if (inserted || iter->second.pixels.size() != expected_size) {
        iter->second.pixels = handle.copy_pixels();
        iter->second.pending_line_start.reset();
        iter->second.last_drag_point.reset();
        iter->second.preview_visible = false;
    }

    return &iter->second;
}

auto PngEditMode::apply_edit(Session const& session, PngMapHandle& handle,
    event::PngMapEditOperation operation, PixelPoint from, PixelPoint to,
    std::size_t thickness) const noexcept -> event::ApplyPngMapEdit::Result {
    auto context       = std::make_unique<event::ApplyPngMapEdit::Context>();
    context->pixels    = session.pixels;
    context->width     = handle.get_width();
    context->height    = handle.get_height();
    context->from      = from;
    context->to        = to;
    context->thickness = thickness;
    context->operation = operation;
    return event::ApplyPngMapEdit::runtime_exec(std::move(context));
}

auto PngEditMode::apply_stroke(Mouse& mouse, PngMapHandle& handle, Session& session,
    event::PngMapEditOperation operation, PixelPoint from, PixelPoint to, std::size_t thickness,
    QString const& verb) noexcept -> bool {
    auto result = apply_edit(session, handle, operation, from, to, thickness);
    if (!result.has_value()) {
        mouse.set_status(mode_status(mouse, QString("%1失败").arg(verb)));
        return false;
    }

    session.pixels = std::move(result.value());
    if (!commit_session(handle, session)) {
        mouse.set_status(mode_status(mouse, QString("%1失败").arg(verb)));
        return false;
    }

    const auto world = from_pixel(handle, to);
    mouse.set_status(mode_status(mouse, QString("%1 %2").arg(verb, format_xyz(world.data()))));
    return true;
}

auto PngEditMode::show_preview(PngMapHandle& handle, Session& session, PixelPoint to,
    std::size_t thickness) noexcept -> bool {
    if (!session.pending_line_start.has_value()) {
        return clear_preview(handle, session);
    }

    auto result = apply_edit(session, handle, event::PngMapEditOperation::Draw,
        *session.pending_line_start, to, thickness);
    if (!result.has_value()) {
        return false;
    }

    session.preview_visible = handle.preview_pixels(result.value());
    if (session.preview_visible) {
        assets.update_renderer();
    }
    return session.preview_visible;
}

auto PngEditMode::clear_preview(PngMapHandle& handle, Session& session) noexcept -> bool {
    if (!session.preview_visible) {
        return true;
    }

    session.preview_visible = false;
    const auto ok           = handle.clear_preview();
    if (ok) {
        assets.update_renderer();
    }
    return ok;
}

auto PngEditMode::commit_session(PngMapHandle& handle, Session const& session) noexcept -> bool {
    if (session.pixels.empty()) {
        return false;
    }

    if (!handle.overwrite_pixels(session.pixels)) {
        return false;
    }

    assets.update_renderer();
    return true;
}

}
