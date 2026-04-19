#pragma once

#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/widget/text-fields.hh>

#include <expected>
#include <string>
#include <string_view>

namespace pcs::gui::working::panels {

auto save_pointcloud_location(std::string const& suggested_name) noexcept
    -> std::expected<std::string, std::string_view>;

auto save_png_map_location(std::string const& suggested_name) noexcept
    -> std::expected<std::string, std::string_view>;

auto export_png_map_directory(std::string const& suggested_name) noexcept
    -> std::expected<std::string, std::string_view>;

auto parse_double_input(creeper::OutlinedTextField& input, double fallback, double minimum) noexcept
    -> double;

auto parse_size_input(creeper::OutlinedTextField& input, std::size_t fallback,
    std::size_t minimum) noexcept -> std::size_t;

auto make_parameter_field(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
    int width, QString const& default_value) noexcept -> creeper::OutlinedTextField*;

auto compact_text_field_measurements() noexcept -> creeper::OutlinedTextField::Measurements;

auto confirm_large_pointcloud_warning(std::uintmax_t bytes) noexcept -> bool;

}
