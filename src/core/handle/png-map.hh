#pragma once

#include "core/map/png-map-data.hh"
#include "core/renderer.hh"
#include "utility/pimpl.hh"

#include <expected>

namespace pcs {

struct PngMapHandle {
    PCS_PIMPL_DEFINITION(PngMapHandle)

public:
    auto set_visibility(bool) noexcept -> void;

    auto load_from_data(PngMapData const&) noexcept -> std::expected<void, std::string_view>;
    auto save_into_filesystem(std::string const& path) const noexcept
        -> std::expected<void, std::string_view>;

    auto get_width() const noexcept -> std::size_t;
    auto get_height() const noexcept -> std::size_t;
    auto get_resolution() const noexcept -> double;
    auto get_plane_z() const noexcept -> double;

    auto attach_renderer(Renderer&) noexcept -> void;
    auto detach_renderer(Renderer&) noexcept -> void;
};

}
