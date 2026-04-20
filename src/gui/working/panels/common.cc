#include "gui/working/panels/common.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/text.hh>

#include <QSignalBlocker>

#include <qfiledialog.h>
#include <qmessagebox.h>

#include <cmath>
#include <filesystem>

using namespace creeper;
namespace pcs::gui::working::panels {

auto save_pointcloud_location(std::string const& suggested_name) noexcept
    -> std::expected<std::string, std::string_view> {
    auto filename = std::filesystem::path(suggested_name);
    if (filename.extension() != ".pcd") {
        filename.replace_extension(".pcd");
    }

    const auto location = QFileDialog::getSaveFileName(
        nullptr, "保存点云", QString::fromStdString(filename.string()), "点云文件 (*.pcd)");

    if (location.isEmpty()) {
        return std::unexpected { "用户取消保存点云" };
    }

    return location.toStdString();
}

auto save_png_map_location(std::string const& suggested_name) noexcept
    -> std::expected<std::string, std::string_view> {
    auto filename = std::filesystem::path(suggested_name);
    if (filename.extension() != ".png") {
        filename.replace_extension(".png");
    }

    const auto location = QFileDialog::getSaveFileName(
        nullptr, "保存 PNG 地图", QString::fromStdString(filename.string()), "PNG 图片 (*.png)");

    if (location.isEmpty()) {
        return std::unexpected { "用户取消保存 PNG 地图" };
    }

    return location.toStdString();
}

auto export_png_map_directory(std::string const& suggested_name) noexcept
    -> std::expected<std::string, std::string_view> {
    auto folder_name = std::filesystem::path(suggested_name).stem();
    if (folder_name.empty()) {
        folder_name = "map";
    }

    const auto parent = QFileDialog::getExistingDirectory(nullptr, "选择导出目录");
    if (parent.isEmpty()) {
        return std::unexpected { "用户取消导出 PNG 地图" };
    }

    return (std::filesystem::path(parent.toStdString()) / folder_name).string();
}

auto parse_double_input(creeper::OutlinedTextField& input, double fallback, double minimum) noexcept
    -> double {
    auto parsed = double { };
    auto ok     = false;
    parsed      = input.text().toDouble(&ok);

    if (!ok || parsed < minimum) {
        parsed = fallback;
    }

    auto normalized = QString::number(parsed, 'f', 6);
    while (normalized.contains('.') && normalized.endsWith('0')) {
        normalized.chop(1);
    }
    if (normalized.endsWith('.')) {
        normalized.chop(1);
    }
    if (normalized == "-0") {
        normalized = "0";
    }

    input.setText(normalized);
    return parsed;
}

auto parse_size_input(creeper::OutlinedTextField& input, std::size_t fallback,
    std::size_t minimum) noexcept -> std::size_t {
    auto ok     = false;
    auto parsed = input.text().toULongLong(&ok);

    if (!ok || parsed < minimum) {
        parsed = fallback;
    }

    input.setText(QString::number(parsed));
    return static_cast<std::size_t>(parsed);
}

auto make_parameter_field(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
    int width, QString const& default_value) noexcept -> creeper::OutlinedTextField* {
    const auto measurements = compact_text_field_measurements();

    return new creeper::OutlinedTextField {
        theme,
        creeper::text_field::pro::Font { font },
        creeper::text_field::pro::FixedWidth { width },
        creeper::widget::pro::Apply {
            [measurements](auto& self) { self.set_measurements(measurements); } },
        creeper::common::pro::Text<creeper::text_field::pro::Token> { default_value },
    };
}

auto compact_text_field_measurements() noexcept -> creeper::OutlinedTextField::Measurements {
    auto measurements = creeper::OutlinedTextField::Measurements { };

    measurements.container_height     = 26;
    measurements.icon_rect_size       = 16;
    measurements.input_rect_size      = 16;
    measurements.label_rect_size      = 12;
    measurements.standard_font_height = 13;

    measurements.col_padding                      = 4;
    measurements.row_padding_without_icons        = 10;
    measurements.row_padding_with_icons           = 8;
    measurements.row_padding_populated_label_text = 0;
    measurements.padding_icons_text               = 8;

    measurements.supporting_text_and_character_counter_top_padding = 2;
    measurements.supporting_text_and_character_counter_row_padding = 8;

    return measurements;
}

auto compact_slider_measurements() noexcept -> creeper::slider::internal::Slider::Measurements {
    auto measurements                   = creeper::slider::internal::Slider::Measurements::Xs();
    measurements.track_height           = 8;
    measurements.handle_height          = 18;
    measurements.handle_width           = 4;
    measurements.track_shape            = 4;
    measurements.label_container_height = 0;
    measurements.label_container_width  = 0;
    return measurements;
}

CompactFieldRow::CompactFieldRow(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
    std::string_view label, int label_width, int field_width,
    QString const& default_value, QString const& placeholder) noexcept {
    input = make_parameter_field(theme, font, field_width, default_value);
    if (!placeholder.isEmpty()) {
        input->setPlaceholderText(placeholder);
    }

    auto* content = new creeper::Row {
        creeper::row::pro::Spacing { 8 },
        creeper::row::pro::Item<creeper::Text> {
            { 0, Qt::AlignVCenter },
            theme,
            creeper::text::pro::Font { font },
            creeper::text::pro::Text { QString::fromStdString(std::string { label }) },
            creeper::text::pro::WordWrap { true },
            creeper::text::pro::Alignment { Qt::AlignVCenter | Qt::AlignLeft },
            creeper::widget::pro::FixedWidth { label_width },
        },
        creeper::row::pro::Item { { 1, Qt::AlignVCenter }, input },
    };

    content->setContentsMargins(0, 0, 0, 0);
    setLayout(content);
}

auto CompactFieldRow::field() const noexcept -> creeper::OutlinedTextField& { return *input; }

CompactDualFieldRow::CompactDualFieldRow(creeper::theme::pro::ThemeManager const& theme,
    QFont const& font, std::string_view label, int label_width, int field_width,
    QString const& first_default, QString const& second_default,
    QString const& first_placeholder, QString const& second_placeholder) noexcept {
    first_input  = make_parameter_field(theme, font, field_width, first_default);
    second_input = make_parameter_field(theme, font, field_width, second_default);
    if (!first_placeholder.isEmpty()) {
        first_input->setPlaceholderText(first_placeholder);
    }
    if (!second_placeholder.isEmpty()) {
        second_input->setPlaceholderText(second_placeholder);
    }

    auto* content = new creeper::Row {
        creeper::row::pro::Spacing { 8 },
        creeper::row::pro::Item<creeper::Text> {
            { 0, Qt::AlignVCenter },
            theme,
            creeper::text::pro::Font { font },
            creeper::text::pro::Text { QString::fromStdString(std::string { label }) },
            creeper::text::pro::WordWrap { true },
            creeper::text::pro::Alignment { Qt::AlignVCenter | Qt::AlignLeft },
            creeper::widget::pro::FixedWidth { label_width },
        },
        creeper::row::pro::Item<creeper::Row> {
            { 1, Qt::AlignVCenter },
            creeper::row::pro::Spacing { 4 },
            creeper::row::pro::Item { first_input },
            creeper::row::pro::Item { second_input },
        },
    };

    content->setContentsMargins(0, 0, 0, 0);
    setLayout(content);
}

auto CompactDualFieldRow::first() const noexcept -> creeper::OutlinedTextField& {
    return *first_input;
}

auto CompactDualFieldRow::second() const noexcept -> creeper::OutlinedTextField& {
    return *second_input;
}

CompactTripleFieldRow::CompactTripleFieldRow(creeper::theme::pro::ThemeManager const& theme,
    QFont const& font, std::string_view label, int label_width, int field_width,
    QString const& first_default, QString const& second_default, QString const& third_default,
    QString const& first_placeholder, QString const& second_placeholder,
    QString const& third_placeholder) noexcept {
    first_input  = make_parameter_field(theme, font, field_width, first_default);
    second_input = make_parameter_field(theme, font, field_width, second_default);
    third_input  = make_parameter_field(theme, font, field_width, third_default);

    if (!first_placeholder.isEmpty()) {
        first_input->setPlaceholderText(first_placeholder);
    }
    if (!second_placeholder.isEmpty()) {
        second_input->setPlaceholderText(second_placeholder);
    }
    if (!third_placeholder.isEmpty()) {
        third_input->setPlaceholderText(third_placeholder);
    }

    auto* content = new creeper::Row {
        creeper::row::pro::Spacing { 8 },
        creeper::row::pro::Item<creeper::Text> {
            { 0, Qt::AlignVCenter },
            theme,
            creeper::text::pro::Font { font },
            creeper::text::pro::Text { QString::fromStdString(std::string { label }) },
            creeper::text::pro::WordWrap { true },
            creeper::text::pro::Alignment { Qt::AlignVCenter | Qt::AlignLeft },
            creeper::widget::pro::FixedWidth { label_width },
        },
        creeper::row::pro::Item<creeper::Row> {
            { 1, Qt::AlignVCenter },
            creeper::row::pro::Spacing { 4 },
            creeper::row::pro::Item { first_input },
            creeper::row::pro::Item { second_input },
            creeper::row::pro::Item { third_input },
        },
    };

    content->setContentsMargins(0, 0, 0, 0);
    setLayout(content);
}

auto CompactTripleFieldRow::first() const noexcept -> creeper::OutlinedTextField& {
    return *first_input;
}

auto CompactTripleFieldRow::second() const noexcept -> creeper::OutlinedTextField& {
    return *second_input;
}

auto CompactTripleFieldRow::third() const noexcept -> creeper::OutlinedTextField& {
    return *third_input;
}

CompactWidgetRow::CompactWidgetRow(creeper::theme::pro::ThemeManager const& theme,
    QFont const& font, std::string_view label, int label_width, QWidget* value_widget) noexcept {
    auto* content = new creeper::Row {
        creeper::row::pro::Spacing { 8 },
        creeper::row::pro::Item<creeper::Text> {
            { 0, Qt::AlignVCenter },
            theme,
            creeper::text::pro::Font { font },
            creeper::text::pro::Text { QString::fromStdString(std::string { label }) },
            creeper::text::pro::WordWrap { true },
            creeper::text::pro::Alignment { Qt::AlignVCenter | Qt::AlignLeft },
            creeper::widget::pro::FixedWidth { label_width },
        },
        creeper::row::pro::Item { { 1, Qt::AlignVCenter }, value_widget },
    };

    content->setContentsMargins(0, 0, 0, 0);
    setLayout(content);
}

ValueSliderRow::ValueSliderRow(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
    std::string_view label, std::shared_ptr<creeper::MutableDouble> value,
    std::function<void()> on_commit) noexcept {
    auto measurement          = creeper::Slider::Measurements::Xs();
    measurement.handle_height = 22;

    auto* content = new creeper::Row {
        creeper::row::pro::Spacing { 8 },
        creeper::row::pro::Item<creeper::Text> {
            theme,
            creeper::MutableTransform {
                [label](creeper::Text& self, double current) {
                    self.setText(QString("%1: %2")
                            .arg(QString::fromStdString(std::string { label }))
                            .arg(QString::number(current, 'f', 2)));
                },
                value,
            },
            creeper::text::pro::Font { font },
        },
        creeper::row::pro::Item<creeper::Slider> {
            { 255 },
            theme,
            creeper::slider::pro::Measurements { measurement },
            creeper::MutableForward {
                creeper::slider::pro::Progress { 0 },
                value,
            },
            creeper::slider::pro::FixedHeight { measurement.minimum_height() },
            creeper::slider::pro::OnValueChangeFinished {
                [value, on_commit = std::move(on_commit)](double v) {
                    *value = v;
                    if (on_commit) {
                        on_commit();
                    }
                },
            },
        },
    };

    content->setContentsMargins(0, 0, 0, 0);
    setLayout(content);
}

AngleSliderFieldRow::AngleSliderFieldRow(creeper::theme::pro::ThemeManager const& theme,
    QFont const& font, QString const& label, int field_width, double default_degrees) noexcept {
    auto chip_font = font;
    chip_font.setPointSize(std::max(8, font.pointSize() - 1));

    value_chip = new creeper::Text {
        theme,
        creeper::text::pro::Font { font },
        creeper::text::pro::Text { "0" },
        creeper::text::pro::Alignment { Qt::AlignCenter },
        creeper::widget::pro::MinimumWidth { 30 },
    };

    value_slider = new creeper::Slider {
        theme,
        creeper::widget::pro::FixedHeight { 24 },
        creeper::widget::pro::MinimumWidth { 150 },
        creeper::slider::pro::Measurements { compact_slider_measurements() },
        creeper::slider::pro::Progress { 0.0 },
    };

    value_field =
        make_parameter_field(theme, font, field_width, QString::number(default_degrees, 'f', 3));

    QObject::connect(value_slider, &creeper::Slider::signal_value_change, this,
        [this](double progress) { sync_from_slider(progress); });
    QObject::connect(value_field, &creeper::OutlinedTextField::editingFinished, this,
        [this]() { sync_from_field(); });

    auto* content = new creeper::Row {
        creeper::row::pro::Spacing { 8 },
        creeper::row::pro::Alignment { Qt::AlignVCenter },
        creeper::row::pro::Item<creeper::FilledCard> {
            theme,
            creeper::card::pro::LevelLowest,
            creeper::card::pro::Layout<creeper::Row> {
                creeper::row::pro::Spacing { 8 },
                creeper::row::pro::Margin { 6 },
                creeper::row::pro::Item<creeper::Text> {
                    theme,
                    creeper::text::pro::Font { chip_font },
                    creeper::text::pro::Text { label },
                    creeper::text::pro::Alignment { Qt::AlignCenter },
                },
                creeper::row::pro::Item { value_chip },
            },
        },
        creeper::row::pro::Item { { 1, Qt::AlignVCenter }, value_slider },
        creeper::row::pro::Item { value_field },
    };

    content->setContentsMargins(0, 0, 0, 0);
    setLayout(content);

    set_degrees(default_degrees);
}

auto AngleSliderFieldRow::set_degrees(double value) noexcept -> void {
    set_degrees_internal(value, true, true);
}

auto AngleSliderFieldRow::degrees() const noexcept -> double { return degrees_value; }

auto AngleSliderFieldRow::field() const noexcept -> creeper::OutlinedTextField& {
    return *value_field;
}

auto AngleSliderFieldRow::slider() const noexcept -> creeper::Slider& { return *value_slider; }

auto AngleSliderFieldRow::sync_from_slider(double progress) noexcept -> void {
    set_degrees_internal(progress * 360.0, false, true);
}

auto AngleSliderFieldRow::sync_from_field() noexcept -> void {
    auto value = parse_double_input(*value_field, degrees_value, 0.0);
    while (value >= 360.0) {
        value -= 360.0;
    }
    set_degrees_internal(value, true, true);
}

auto AngleSliderFieldRow::set_degrees_internal(
    double value, bool update_slider, bool update_field) noexcept -> void {
    while (value < 0.0) {
        value += 360.0;
    }
    while (value >= 360.0) {
        value -= 360.0;
    }

    degrees_value = value;
    value_chip->setText(QString::number(static_cast<int>(std::round(degrees_value))));

    if (update_slider) {
        const auto blocker = QSignalBlocker { value_slider };
        value_slider->set_progress(degrees_value / 360.0);
    }

    if (update_field) {
        const auto blocker = QSignalBlocker { value_field };
        value_field->setText(QString::number(degrees_value, 'f', 3));
    }
}

auto confirm_large_pointcloud_warning(std::uintmax_t bytes) noexcept -> bool {
    const auto size_mb = static_cast<double>(bytes) / (1024.0 * 1024.0);
    const auto message = QString("当前点云大小为 %1 MB（超过 100 MB），生成 PNG "
                                 "地图可能耗时较长，是否继续？")
                             .arg(size_mb, 0, 'f', 1);

    const auto answer = QMessageBox::warning(
        nullptr, "大体积点云提示", message, QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

    return answer == QMessageBox::Yes;
}

}
