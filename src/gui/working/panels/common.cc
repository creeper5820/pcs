#include "gui/working/panels/common.hh"

#include <qfiledialog.h>
#include <qmessagebox.h>

#include <filesystem>

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
