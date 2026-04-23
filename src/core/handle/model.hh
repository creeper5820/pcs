#pragma once

#include "core/map/model-data.hh"
#include "core/renderer.hh"
#include "utility/pimpl.hh"

#include <expected>
#include <string_view>
#include <tuple>

namespace pcs {

struct ModelHandle {
    PCS_PIMPL_DEFINITION(ModelHandle)

public:
    static constexpr std::string_view kKind = "model";

    using Position = std::tuple<double, double, double>;

    auto set_position(double x, double y, double z) noexcept -> void;
    auto get_position() const noexcept -> Position;

    auto set_overall_color(double r, double g, double b) noexcept -> void;
    auto get_overall_color() const noexcept -> std::tuple<double, double, double>;

    auto get_points_size() const noexcept -> std::size_t;
    auto get_polys_size() const noexcept -> std::size_t;

    auto set_visibility(bool) noexcept -> void;

    auto load_from_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view>;
    auto save_into_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view>;

    auto model_data() const noexcept -> ModelData const&;

    auto clone(std::string const& target_name) const noexcept
        -> std::expected<std::unique_ptr<ModelHandle>, std::string>;

    auto attach_renderer(Renderer&) noexcept -> void;

    auto detach_renderer(Renderer&) noexcept -> void;
};

}
