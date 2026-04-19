#pragma once

#include "core/map/png-map-data.hh"
#include "core/renderer.hh"
#include "utility/pimpl.hh"

#include <expected>
#include <optional>
#include <tuple>

namespace pcs {

struct PngMapHandle {
    PCS_PIMPL_DEFINITION(PngMapHandle)

public:
    using Position = std::tuple<double, double, double>;

    auto set_visibility(bool) noexcept -> void;

    auto load_from_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view>;
    auto load_from_data(PngMapData const&) noexcept -> std::expected<void, std::string_view>;
    auto save_into_filesystem(std::string const& path) const noexcept
        -> std::expected<void, std::string_view>;

    auto get_width() const noexcept -> std::size_t;
    auto get_height() const noexcept -> std::size_t;
    auto get_resolution() const noexcept -> double;
    auto get_plane_z() const noexcept -> double;
    auto get_origin_x() const noexcept -> double;
    auto get_origin_y() const noexcept -> double;

    auto copy_pixels() const noexcept -> std::vector<std::uint8_t>;
    auto overwrite_pixels(std::vector<std::uint8_t> const&) noexcept -> bool;
    auto preview_pixels(std::vector<std::uint8_t> const&) noexcept -> bool;
    auto clear_preview() noexcept -> bool;

    auto pick_plane_position(Renderer&, int display_x, int display_y) const noexcept
        -> std::optional<Position>;

    auto attach_renderer(Renderer&) noexcept -> void;
    auto detach_renderer(Renderer&) noexcept -> void;
};

}
