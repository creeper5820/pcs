#pragma once

#include "core/map/png-map-data.hh"
#include "core/map/png-map-transform.hh"
#include "core/renderer.hh"
#include "utility/pimpl.hh"

#include <expected>
#include <optional>
#include <string_view>
#include <tuple>

namespace pcs {

struct PngMapHandle {
    PCS_PIMPL_DEFINITION(PngMapHandle)

public:
    static constexpr std::string_view kKind = "png-map";

    using Position = std::tuple<double, double, double>;

    auto set_visibility(bool) noexcept -> void;
    auto set_frame_visibility(bool) noexcept -> void;
    auto frame_visibility() const noexcept -> bool;
    auto set_source_area_visibility(bool) noexcept -> void;
    auto source_area_visibility() const noexcept -> bool;

    auto load_from_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view>;
    auto load_from_data(PngMapData const&) noexcept -> std::expected<void, std::string_view>;
    auto save_into_filesystem(std::string const& path) const noexcept
        -> std::expected<void, std::string_view>;
    auto export_to_ros_directory(std::string const& directory) const noexcept
        -> std::expected<void, std::string_view>;

    auto clone(std::string const& target_name) const noexcept
        -> std::expected<std::unique_ptr<PngMapHandle>, std::string>;

    auto get_width() const noexcept -> std::size_t;
    auto get_height() const noexcept -> std::size_t;
    auto get_resolution() const noexcept -> double;
    auto get_plane_z() const noexcept -> double;
    auto get_origin_x() const noexcept -> double;
    auto get_origin_y() const noexcept -> double;
    auto get_z_area_start() const noexcept -> double;
    auto get_z_area_end() const noexcept -> double;
    auto get_frame_config() const noexcept -> PngMapFrameConfig;
    auto set_frame_config(PngMapFrameConfig const&) noexcept -> bool;
    auto transform_view() const noexcept -> PngMapTransformView;

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
