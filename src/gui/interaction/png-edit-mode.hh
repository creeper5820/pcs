#pragma once

#include "core/assets.hh"
#include "core/events/process/png-map-edit.hh"
#include "core/map/png-edit-ops.hh"
#include "core/renderer.hh"
#include "core/runtime.hh"
#include "gui/interaction/mouse.hh"

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace pcs::gui::interaction {

class PngEditMode final : public MouseMode {
public:
    PngEditMode(Renderer&, AssetsManager&, Runtime&) noexcept;

    auto id() const noexcept -> MouseModeId override;
    auto on_init(Mouse&) noexcept -> void override;
    auto on_exit(Mouse&) noexcept -> void override;
    auto supports_selection(std::optional<MouseSelection> const&) const noexcept -> bool override;
    auto allows_camera_interaction(Mouse const&) const noexcept -> bool override;

private:
    struct Session {
        std::vector<std::uint8_t> pixels;
        std::optional<PixelPoint> pending_line_start;
        std::optional<PixelPoint> last_drag_point;
        bool preview_visible = false;
    };

    struct EditContext {
        std::string const& asset_id;
        PngMapHandle& handle;
        Session& session;
        std::optional<PixelPoint> pixel;
    };

    struct ToolHandler {
        std::function<void(PngEditMode&, Mouse&, MouseEvent const&, EditContext&)> on_move;
        std::function<void(PngEditMode&, Mouse&, MouseEvent const&, EditContext&)> on_lclick;
        std::function<void(PngEditMode&, Mouse&, MouseEvent const&, EditContext&)> on_rclick;
    };

    auto register_tool_handler(PngEditTool, ToolHandler) noexcept -> void;
    auto resolve_edit_context(Mouse&, MouseEvent const&) noexcept -> std::optional<EditContext>;
    auto on_move(Mouse&, MouseEvent const&) noexcept -> void;
    auto on_lclick(Mouse&, MouseEvent const&) noexcept -> void;
    auto on_rclick(Mouse&, MouseEvent const&) noexcept -> void;

    auto pick_pixel(std::string const& asset_id, MouseEvent const&) const noexcept
        -> std::optional<PixelPoint>;
    auto ensure_session(std::string const& asset_id, PngMapHandle&) noexcept -> Session*;
    auto apply_stroke(Mouse&, PngMapHandle&, Session&, event::PngMapEditOperation, PixelPoint from,
        PixelPoint to, std::size_t thickness, QString const& verb) noexcept -> bool;
    auto apply_edit(Session const&, PngMapHandle&, event::PngMapEditOperation, PixelPoint from,
        PixelPoint to, std::size_t thickness) const noexcept -> event::ApplyPngMapEdit::Result;
    auto show_preview(PngMapHandle&, Session&, PixelPoint to, std::size_t thickness) noexcept
        -> bool;
    auto clear_preview(PngMapHandle&, Session&) noexcept -> bool;
    auto commit_session(PngMapHandle&, Session const&) noexcept -> bool;

    Renderer& renderer;
    AssetsManager& assets;
    Runtime& runtime;
    std::unordered_map<std::string, Session> sessions;
    std::unordered_map<PngEditTool, ToolHandler> tool_handlers;
};

}
