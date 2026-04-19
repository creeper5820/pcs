#pragma once

#include "core/assets.hh"

#include <qnamespace.h>
#include <qstring.h>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace pcs::gui::interaction {

enum class MouseModeId {
    None,
    Picker,
    PngEdit,
};

enum class PngEditTool {
    Free,
    Line,
    Point,
    Erase,
};

struct MouseEvent {
    int x = 0;
    int y = 0;
    Qt::KeyboardModifiers modifiers { };
    Qt::MouseButtons buttons { };
};

struct MouseSelection {
    std::string id;
    pcs::AssetKind kind = pcs::AssetKind::Pointcloud;
};

class Mouse;

struct MouseMode {
    virtual ~MouseMode() = default;

    virtual auto id() const noexcept -> MouseModeId = 0;
    virtual auto on_init(Mouse&) noexcept -> void { }
    virtual auto on_exit(Mouse&) noexcept -> void { }
    virtual auto supports_selection(std::optional<MouseSelection> const&) const noexcept -> bool {
        return true;
    }
    virtual auto allows_camera_interaction(Mouse const&) const noexcept -> bool { return true; }
};

class Mouse final {
public:
    using Handler = std::function<void(MouseEvent const&)>;
    using Token   = std::size_t;

    auto register_mode(std::unique_ptr<MouseMode>) noexcept -> void;

    auto set_mode(MouseModeId) noexcept -> void;
    auto mode() const noexcept -> MouseModeId;
    auto allows_camera_interaction() const noexcept -> bool;

    auto on_move(Handler) noexcept -> Token;
    auto on_lclick(Handler) noexcept -> Token;
    auto on_rclick(Handler) noexcept -> Token;
    auto off(Token) noexcept -> bool;
    auto clear_handlers() noexcept -> void;

    auto emit_move(MouseEvent const&) noexcept -> void;
    auto emit_lclick(MouseEvent const&) noexcept -> void;
    auto emit_rclick(MouseEvent const&) noexcept -> void;

    auto set_status_sink(std::function<void(QString const&)>) noexcept -> void;
    auto set_status(QString) noexcept -> void;
    auto status() const noexcept -> QString const&;

    auto set_mode_sink(std::function<void(MouseModeId)>) noexcept -> void;
    auto set_png_edit_tool_sink(std::function<void(PngEditTool)>) noexcept -> void;

    auto set_png_edit_line_width(std::size_t) noexcept -> void;
    auto png_edit_line_width() const noexcept -> std::size_t;

    auto set_png_edit_point_size(std::size_t) noexcept -> void;
    auto png_edit_point_size() const noexcept -> std::size_t;

    auto set_png_edit_erase_size(std::size_t) noexcept -> void;
    auto png_edit_erase_size() const noexcept -> std::size_t;

    auto set_png_edit_tool(PngEditTool) noexcept -> void;
    auto png_edit_tool() const noexcept -> PngEditTool;

    auto set_selected_asset(std::string id, pcs::AssetKind kind) noexcept -> void;
    auto clear_selected_asset() noexcept -> void;
    auto selected_asset() const noexcept -> std::optional<MouseSelection> const&;

private:
    struct HandlerEntry {
        MouseModeId mode = MouseModeId::None;
        Handler callback;
    };

    auto next_token() noexcept -> Token;
    auto add_handler(std::unordered_map<Token, HandlerEntry>&, Handler) noexcept -> Token;
    auto emit_handlers(
        std::unordered_map<Token, HandlerEntry> const&, MouseEvent const&) const noexcept -> void;

    std::unordered_map<MouseModeId, std::unique_ptr<MouseMode>> modes;
    MouseModeId current_mode = MouseModeId::None;

    std::unordered_map<Token, HandlerEntry> move_handlers;
    std::unordered_map<Token, HandlerEntry> lclick_handlers;
    std::unordered_map<Token, HandlerEntry> rclick_handlers;

    Token last_token = 0;

    QString status_text = "就绪";
    std::function<void(QString const&)> status_sink;
    std::function<void(MouseModeId)> mode_sink;
    std::function<void(PngEditTool)> png_edit_tool_sink;

    std::size_t png_edit_line_width_px = 3;
    std::size_t png_edit_point_size_px = 4;
    std::size_t png_edit_erase_size_px = 10;
    PngEditTool png_edit_tool_mode     = PngEditTool::Free;

    std::optional<MouseSelection> asset;
};

}
