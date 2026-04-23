#pragma once

#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/utility/wrapper/mutable-value.hh>
#include <creeper-qt/widget/sliders.hh>
#include <creeper-qt/widget/text-fields.hh>
#include <creeper-qt/widget/text.hh>

#include <QWidget>

#include <expected>
#include <functional>
#include <memory>
#include <string>
#include <string_view>

namespace pcs::gui::working::panels {

class CompactFieldRow final : public QWidget {
public:
    CompactFieldRow(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
        std::string_view label, int label_width, int field_width, QString const& default_value,
        QString const& placeholder = { }) noexcept;

    auto field() const noexcept -> creeper::OutlinedTextField&;

private:
    creeper::OutlinedTextField* input = nullptr;
};

class CompactDualFieldRow final : public QWidget {
public:
    CompactDualFieldRow(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
        std::string_view label, int label_width, int field_width, QString const& first_default,
        QString const& second_default, QString const& first_placeholder = { },
        QString const& second_placeholder = { }) noexcept;

    auto first() const noexcept -> creeper::OutlinedTextField&;
    auto second() const noexcept -> creeper::OutlinedTextField&;

private:
    creeper::OutlinedTextField* first_input  = nullptr;
    creeper::OutlinedTextField* second_input = nullptr;
};

class CompactTripleFieldRow final : public QWidget {
public:
    CompactTripleFieldRow(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
        std::string_view label, int label_width, int field_width, QString const& first_default,
        QString const& second_default, QString const& third_default,
        QString const& first_placeholder = { }, QString const& second_placeholder = { },
        QString const& third_placeholder = { }) noexcept;

    auto first() const noexcept -> creeper::OutlinedTextField&;
    auto second() const noexcept -> creeper::OutlinedTextField&;
    auto third() const noexcept -> creeper::OutlinedTextField&;

private:
    creeper::OutlinedTextField* first_input  = nullptr;
    creeper::OutlinedTextField* second_input = nullptr;
    creeper::OutlinedTextField* third_input  = nullptr;
};

class CompactWidgetRow final : public QWidget {
public:
    CompactWidgetRow(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
        std::string_view label, int label_width, QWidget* value_widget) noexcept;
};

class ValueSliderRow final : public QWidget {
public:
    ValueSliderRow(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
        std::string_view label, std::shared_ptr<creeper::MutableDouble> value,
        std::function<void()> on_commit) noexcept;
};

class AngleSliderFieldRow final : public QWidget {
public:
    AngleSliderFieldRow(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
        QString const& label, int field_width, double default_degrees) noexcept;

    auto set_degrees(double value) noexcept -> void;
    auto degrees() const noexcept -> double;

    auto field() const noexcept -> creeper::OutlinedTextField&;
    auto slider() const noexcept -> creeper::Slider&;

private:
    auto sync_from_slider(double progress) noexcept -> void;
    auto sync_from_field() noexcept -> void;
    auto set_degrees_internal(double value, bool update_slider, bool update_field) noexcept -> void;

    creeper::Text* value_chip               = nullptr;
    creeper::Slider* value_slider           = nullptr;
    creeper::OutlinedTextField* value_field = nullptr;
    double degrees_value                    = 0.0;
};

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

auto compact_slider_measurements() noexcept -> creeper::Slider::Measurements;

auto confirm_large_pointcloud_warning(std::uintmax_t bytes) noexcept -> bool;

}
