#include "gui/interaction/mouse.hh"

#include <utility>

namespace pcs::gui::interaction {

auto Mouse::register_mode(std::unique_ptr<MouseMode> mode) noexcept -> void {
    if (mode == nullptr) {
        return;
    }

    modes[mode->id()] = std::move(mode);
}

auto Mouse::set_mode(MouseModeId mode) noexcept -> void {
    if (current_mode == mode) {
        return;
    }

    if (auto iter = modes.find(current_mode); iter != modes.end() && iter->second != nullptr) {
        iter->second->on_exit(*this);
    }

    clear_handlers();
    current_mode = mode;

    if (auto iter = modes.find(current_mode); iter != modes.end() && iter->second != nullptr) {
        iter->second->on_init(*this);
    }

    if (mode_sink) {
        mode_sink(current_mode);
    }
}

auto Mouse::mode() const noexcept -> MouseModeId { return current_mode; }

auto Mouse::allows_camera_interaction() const noexcept -> bool {
    auto iter = modes.find(current_mode);
    if (iter == modes.end() || iter->second == nullptr) {
        return true;
    }

    return iter->second->allows_camera_interaction(*this);
}

auto Mouse::on_move(Handler handler) noexcept -> Token {
    return add_handler(move_handlers, std::move(handler));
}

auto Mouse::on_lclick(Handler handler) noexcept -> Token {
    return add_handler(lclick_handlers, std::move(handler));
}

auto Mouse::on_rclick(Handler handler) noexcept -> Token {
    return add_handler(rclick_handlers, std::move(handler));
}

auto Mouse::off(Token token) noexcept -> bool {
    if (move_handlers.erase(token) > 0) {
        return true;
    }
    if (lclick_handlers.erase(token) > 0) {
        return true;
    }
    if (rclick_handlers.erase(token) > 0) {
        return true;
    }
    return false;
}

auto Mouse::clear_handlers() noexcept -> void {
    move_handlers.clear();
    lclick_handlers.clear();
    rclick_handlers.clear();
}

auto Mouse::emit_move(MouseEvent const& event) noexcept -> void {
    emit_handlers(move_handlers, event);
}

auto Mouse::emit_lclick(MouseEvent const& event) noexcept -> void {
    emit_handlers(lclick_handlers, event);
}

auto Mouse::emit_rclick(MouseEvent const& event) noexcept -> void {
    emit_handlers(rclick_handlers, event);
}

auto Mouse::set_status_sink(std::function<void(QString const&)> sink) noexcept -> void {
    status_sink = std::move(sink);
    if (status_sink) {
        status_sink(status_text);
    }
}

auto Mouse::set_status(QString text) noexcept -> void {
    status_text = std::move(text);
    if (status_sink) {
        status_sink(status_text);
    }
}

auto Mouse::status() const noexcept -> QString const& { return status_text; }

auto Mouse::set_mode_sink(std::function<void(MouseModeId)> sink) noexcept -> void {
    mode_sink = std::move(sink);
    if (mode_sink) {
        mode_sink(current_mode);
    }
}

auto Mouse::set_png_edit_tool_sink(std::function<void(PngEditTool)> sink) noexcept -> void {
    png_edit_tool_sink = std::move(sink);
    if (png_edit_tool_sink) {
        png_edit_tool_sink(png_edit_tool_mode);
    }
}

auto Mouse::set_png_edit_line_width(std::size_t width) noexcept -> void {
    png_edit_line_width_px = std::max<std::size_t>(width, 1);
}

auto Mouse::png_edit_line_width() const noexcept -> std::size_t { return png_edit_line_width_px; }

auto Mouse::set_png_edit_point_size(std::size_t size) noexcept -> void {
    png_edit_point_size_px = std::max<std::size_t>(size, 1);
}

auto Mouse::png_edit_point_size() const noexcept -> std::size_t { return png_edit_point_size_px; }

auto Mouse::set_png_edit_erase_size(std::size_t size) noexcept -> void {
    png_edit_erase_size_px = std::max<std::size_t>(size, 1);
}

auto Mouse::png_edit_erase_size() const noexcept -> std::size_t { return png_edit_erase_size_px; }

auto Mouse::set_png_edit_tool(PngEditTool tool) noexcept -> void {
    png_edit_tool_mode = tool;
    if (png_edit_tool_sink) {
        png_edit_tool_sink(png_edit_tool_mode);
    }
}

auto Mouse::png_edit_tool() const noexcept -> PngEditTool { return png_edit_tool_mode; }

auto Mouse::set_selected_asset(std::string id, pcs::AssetKind kind) noexcept -> void {
    asset = MouseSelection {
        .id   = std::move(id),
        .kind = kind,
    };

    auto iter = modes.find(current_mode);
    if (iter != modes.end() && iter->second != nullptr && !iter->second->supports_selection(asset)) {
        set_mode(MouseModeId::None);
    }
}

auto Mouse::clear_selected_asset() noexcept -> void {
    asset.reset();

    auto iter = modes.find(current_mode);
    if (iter != modes.end() && iter->second != nullptr && !iter->second->supports_selection(asset)) {
        set_mode(MouseModeId::None);
    }
}

auto Mouse::selected_asset() const noexcept -> std::optional<MouseSelection> const& { return asset; }

auto Mouse::next_token() noexcept -> Token {
    ++last_token;
    return last_token;
}

auto Mouse::add_handler(
    std::unordered_map<Token, HandlerEntry>& handlers, Handler callback) noexcept -> Token {
    auto token = next_token();
    handlers.emplace(token,
        HandlerEntry {
            .mode     = current_mode,
            .callback = std::move(callback),
        });
    return token;
}

auto Mouse::emit_handlers(std::unordered_map<Token, HandlerEntry> const& handlers,
    MouseEvent const& event) const noexcept -> void {
    for (auto const& [_, entry] : handlers) {
        if (entry.callback) {
            entry.callback(event);
        }
    }
}

}
