#include "gui/working/action-panels.hh"

#include "core/events/process/pointcloud-to-png-map.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/widget/sliders.hh>
#include <creeper-qt/widget/text.hh>

#include <QCoreApplication>
#include <QMetaObject>
#include <QPointer>
#include <QSizePolicy>
#include <QThreadPool>

#include <qfiledialog.h>
#include <qmessagebox.h>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <tuple>

namespace pcs::gui::working {

namespace {

    constexpr auto kLargePointcloudBytes = std::uintmax_t { 100 } * 1024U * 1024U;

    namespace text_field_props {
        using Token = text_field::pro::Token;

        using Measurements = SetterProp<Token, text_field::internal::BasicTextField::Measurements,
            [](auto& self, const auto& value) { self.set_measurements(value); }>;

        using Text =
            SetterProp<Token, QString, [](auto& self, const auto& value) { self.setText(value); }>;
    }

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

        const auto location = QFileDialog::getSaveFileName(nullptr, "保存 PNG 地图",
            QString::fromStdString(filename.string()), "PNG 图片 (*.png)");

        if (location.isEmpty()) {
            return std::unexpected { "用户取消保存 PNG 地图" };
        }

        return location.toStdString();
    }

    auto parse_double_input(text_field::internal::BasicTextField* input, double fallback,
        double minimum) noexcept -> double {
        auto parsed = double { };
        auto ok     = false;
        parsed      = input->text().toDouble(&ok);

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

        input->setText(normalized);
        return parsed;
    }

    auto parse_size_input(text_field::internal::BasicTextField* input, std::size_t fallback,
        std::size_t minimum) noexcept -> std::size_t {
        auto ok     = false;
        auto parsed = input->text().toULongLong(&ok);

        if (!ok || parsed < minimum) {
            parsed = fallback;
        }

        input->setText(QString::number(parsed));
        return static_cast<std::size_t>(parsed);
    }

    auto compact_text_field_measurements() noexcept
        -> text_field::internal::BasicTextField::Measurements {
        auto measurement                 = text_field::internal::BasicTextField::Measurements { };
        measurement.container_height     = 34;
        measurement.icon_rect_size       = 16;
        measurement.input_rect_size      = 16;
        measurement.label_rect_size      = 12;
        measurement.standard_font_height = 12;
        measurement.col_padding          = 6;
        measurement.row_padding_without_icons        = 10;
        measurement.row_padding_with_icons           = 10;
        measurement.row_padding_populated_label_text = 0;
        return measurement;
    }

    auto make_parameter_field(theme::pro::ThemeManager const& theme, QFont const& font, int width,
        text_field::internal::BasicTextField::Measurements const& measurements,
        QString const& default_value) noexcept -> OutlinedTextField* {
        return new OutlinedTextField {
            theme,
            widget::pro::Font { font },
            widget::pro::FixedWidth { width },
            text_field_props::Measurements { measurements },
            text_field_props::Text { default_value },
        };
    }

    auto confirm_large_pointcloud_warning(std::uintmax_t bytes) noexcept -> bool {
        const auto size_mb = static_cast<double>(bytes) / (1024.0 * 1024.0);
        const auto message = QString("当前点云大小为 %1 MB（超过 100 MB），生成 PNG "
                                     "地图可能耗时较长，是否继续？")
                                 .arg(size_mb, 0, 'f', 1);

        const auto answer = QMessageBox::warning(nullptr, "大体积点云提示", message,
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

        return answer == QMessageBox::Yes;
    }

}

PointcloudActionPanel::PointcloudActionPanel(ActionPanelContext context, QFont const& font)
    : context { std::move(context) } {
    const auto theme = theme::pro::ThemeManager { *this->context.manager };

    const auto update_color = [this] {
        if (selected_asset_id.empty()) {
            return;
        }

        if (auto result = this->context.assets->get_pointcloud_handle(selected_asset_id)) {
            auto* handle = result.value();
            handle->set_overall_color(
                *color_channels[0], *color_channels[1], *color_channels[2], *color_channels[3]);
            this->context.assets->update_renderer();
        }
    };

    const auto make_slider = [=](std::shared_ptr<MutableDouble> channel, std::string_view name) {
        auto measurement          = Slider::Measurements::Xs();
        measurement.handle_height = 22;

        return new Row {
            row::pro::Spacing { 8 },
            row::pro::Item<Text> {
                theme,
                MutableTransform {
                    [name](Text& self, double value) {
                        self.setText(QString("%1: %2")
                                .arg(QString::fromStdString(std::string { name }))
                                .arg(QString::number(value, 'f', 2)));
                    },
                    channel,
                },
                text::pro::Font { font },
            },
            row::pro::Item<Slider> {
                { 255 },
                theme,
                slider::pro::Measurements { measurement },
                MutableForward {
                    slider::pro::Progress { 0 },
                    channel,
                },
                slider::pro::FixedHeight { measurement.minimum_height() },
                slider::pro::OnValueChangeFinished {
                    [=](double v) {
                        *channel = v;
                        update_color();
                    },
                },
            },
        };
    };

    const auto measurements = compact_text_field_measurements();

    const auto make_input_row = [=](std::string_view label,
                                    text_field::internal::BasicTextField*& input,
                                    QString const& default_value) {
        auto* field = make_parameter_field(theme, font, 116, measurements, default_value);
        input       = field;

        return new Row {
            row::pro::Spacing { 8 },
            row::pro::Item<Text> {
                { 0, Qt::AlignVCenter },
                theme,
                text::pro::Font { font },
                text::pro::Text { QString::fromStdString(std::string { label }) },
                text::pro::WordWrap { true },
                text::pro::Alignment { Qt::AlignVCenter | Qt::AlignLeft },
                widget::pro::FixedWidth { 92 },
            },
            row::pro::Item { { 1, Qt::AlignVCenter }, field },
        };
    };

    const auto make_dual_input_row = [=](std::string_view label,
                                         text_field::internal::BasicTextField*& first,
                                         QString const& first_default,
                                         text_field::internal::BasicTextField*& second,
                                         QString const& second_default) {
        auto* first_field = make_parameter_field(theme, font, 56, measurements, first_default);
        first             = first_field;

        auto* second_field = make_parameter_field(theme, font, 56, measurements, second_default);
        second             = second_field;

        return new Row {
            row::pro::Spacing { 8 },
            row::pro::Item<Text> {
                { 0, Qt::AlignVCenter },
                theme,
                text::pro::Font { font },
                text::pro::Text { QString::fromStdString(std::string { label }) },
                text::pro::WordWrap { true },
                text::pro::Alignment { Qt::AlignVCenter | Qt::AlignLeft },
                widget::pro::FixedWidth { 92 },
            },
            row::pro::Item<Row> {
                { 1, Qt::AlignVCenter },
                row::pro::Spacing { 4 },
                row::pro::Item { first_field },
                row::pro::Item { second_field },
            },
        };
    };

    const auto set_generate_busy = [this](bool busy) {
        if (generate_button == nullptr) {
            return;
        }

        generate_button->setDisabled(busy);
        generate_button->setText(busy ? QString::fromUtf8("生成中") : "生成 PNG 地图");
    };

    const auto should_continue_large_generation = [this](std::size_t points_count) -> bool {
        const auto path = this->context.assets->get_asset_path(selected_asset_id).value_or("");
        if (!path.empty() && path != "<memory>") {
            auto error      = std::error_code { };
            const auto size = std::filesystem::file_size(path, error);
            if (!error && size > kLargePointcloudBytes) {
                return confirm_large_pointcloud_warning(size);
            }
            return true;
        }

        constexpr auto bytes_per_point = sizeof(double) * 3U;
        const auto estimated_size = static_cast<std::uintmax_t>(points_count) * bytes_per_point;
        if (estimated_size > kLargePointcloudBytes) {
            return confirm_large_pointcloud_warning(estimated_size);
        }

        return true;
    };

    save_button = new OutlinedButton {
        theme,
        widget::pro::FixedHeight { 36 },
        widget::pro::MinimumWidth { 180 },
        button::pro::Text { "保存点云" },
    };

    generate_button = new OutlinedButton {
        theme,
        widget::pro::FixedHeight { 36 },
        widget::pro::MinimumWidth { 180 },
        button::pro::Text { "生成 PNG 地图" },
    };

    QObject::connect(save_button, &OutlinedButton::clicked, [this](bool) {
        if (selected_asset_id.empty()) {
            return;
        }

        const auto suggested =
            this->context.assets->get_asset_name(selected_asset_id).value_or("pointcloud.pcd");
        if (auto location = save_pointcloud_location(suggested)) {
            const auto result =
                this->context.assets->save_pointcloud_asset(selected_asset_id, *location);
            if (!result.has_value()) {
                spdlog::error("保存点云资产失败: {}", result.error());
                return;
            }

            this->context.refresh_assets_list();
            this->context.select_asset(selected_asset_id);
        }
    });

    QObject::connect(generate_button, &OutlinedButton::clicked,
        [this, set_generate_busy, should_continue_large_generation](bool) {
            if (selected_asset_id.empty()) {
                return;
            }
            if (generate_button != nullptr && !generate_button->isEnabled()) {
                return;
            }

            const auto handle_result =
                this->context.assets->get_pointcloud_handle(selected_asset_id);
            if (!handle_result.has_value()) {
                spdlog::error("点云句柄不可用");
                return;
            }

            auto source_points = handle_result.value()->get_positions();
            if (source_points.empty()) {
                spdlog::error("点云为空");
                return;
            }

            if (!should_continue_large_generation(source_points.size())) {
                return;
            }

            auto params             = pcs::PngMapParameters { };
            params.resolution       = parse_double_input(resolution_input, 0.1, 0.01);
            params.points_limit     = parse_size_input(points_limit_input, 5, 1);
            params.height_limit     = parse_double_input(height_limit_input, 0.1, 0.0);
            params.influence_radius = parse_double_input(influence_radius_input, 0.08, 0.0);
            params.z_area_start     = parse_double_input(z_area_start_input, 0.0, 0.0);
            params.z_area_end       = parse_double_input(z_area_end_input, 0.2, 0.0);

            set_generate_busy(true);

            auto context_copy    = this->context;
            auto source_asset_id = std::string { selected_asset_id };
            auto button_guard    = QPointer<OutlinedButton> { generate_button };
            auto panel_guard     = QPointer<QWidget> { root };

            QThreadPool::globalInstance()->start([context_copy,
                                                     source_asset_id = std::move(source_asset_id),
                                                     source_points   = std::move(source_points),
                                                     params, button_guard, panel_guard]() mutable {
                auto event_context =
                    std::make_unique<pcs::event::ConvertPointcloudToPngMap::Context>();
                event_context->points     = std::move(source_points);
                event_context->parameters = params;

                auto compute_result =
                    std::make_shared<pcs::event::ConvertPointcloudToPngMap::Result>(
                        pcs::event::ConvertPointcloudToPngMap::runtime_exec(
                            std::move(event_context)));

                QMetaObject::invokeMethod(
                    QCoreApplication::instance(),
                    [context_copy, source_asset_id = std::move(source_asset_id), compute_result,
                        button_guard, panel_guard]() {
                        if (button_guard) {
                            button_guard->setDisabled(false);
                            button_guard->setText("生成 PNG 地图");
                        }

                        if (!panel_guard) {
                            return;
                        }

                        if (!compute_result->has_value()) {
                            spdlog::error("点云生成 PNG 地图失败: {}", compute_result->error());
                            return;
                        }

                        const auto create_result = context_copy.assets->upsert_generated_png_map(
                            source_asset_id, compute_result->value());
                        if (!create_result.has_value()) {
                            spdlog::error("创建 PNG 地图资产失败: {}", create_result.error());
                            return;
                        }

                        context_copy.refresh_assets_list();
                        context_copy.select_asset(*create_result);
                    },
                    Qt::QueuedConnection);
            });
        });

    root = new FilledCard {
        theme,
        card::pro::LevelLow,
        card::pro::Layout<Col> {
            col::pro::Alignment { Qt::AlignTop },
            col::pro::Margin { 10 },
            col::pro::Spacing { 10 },
            col::pro::Item<Text> {
                theme,
                text::pro::Font { font },
                text::pro::Text { "点云操作" },
                text::pro::Alignment { Qt::AlignHCenter },
            },
            col::pro::Item<FilledCard> {
                theme,
                card::pro::LevelLowest,
                card::pro::Layout<Col> {
                    col::pro::Margin { 8 },
                    col::pro::Spacing { 4 },
                    col::pro::Item { make_slider(color_channels[0], "R") },
                    col::pro::Item { make_slider(color_channels[1], "G") },
                    col::pro::Item { make_slider(color_channels[2], "B") },
                    col::pro::Item { make_slider(color_channels[3], "A") },
                },
            },
            col::pro::Item<FilledCard> {
                theme,
                card::pro::LevelLowest,
                card::pro::Layout<Col> {
                    col::pro::Margin { 8 },
                    col::pro::Spacing { 4 },
                    col::pro::Item { make_input_row("分辨率（m）", resolution_input, "0.1") },
                    col::pro::Item { make_input_row("有效点云数（点）", points_limit_input, "5") },
                    col::pro::Item { make_input_row("有效高度差（m）", height_limit_input, "0.1") },
                    col::pro::Item {
                        make_input_row("影响半径（m）", influence_radius_input, "0.08") },
                    col::pro::Item { make_dual_input_row(
                        "Z 区间（m）", z_area_start_input, "0", z_area_end_input, "0.2") },
                    col::pro::Item { generate_button },
                },
            },
            col::pro::Item { save_button },
        },
    };
}

auto PointcloudActionPanel::widget() const noexcept -> QWidget* { return root; }

auto PointcloudActionPanel::bind_asset(std::string const& id) noexcept -> void {
    selected_asset_id = id;

    if (auto result = context.assets->get_pointcloud_handle(selected_asset_id)) {
        auto* handle = result.value();
        std::tie(*color_channels[0], *color_channels[1], *color_channels[2], *color_channels[3]) =
            handle->get_overall_color();
    }
}

auto PointcloudActionPanel::clear() noexcept -> void { selected_asset_id.clear(); }

ModelActionPanel::ModelActionPanel(ActionPanelContext context, QFont const& font)
    : context { std::move(context) } {
    const auto theme        = theme::pro::ThemeManager { *this->context.manager };
    const auto measurements = compact_text_field_measurements();

    const auto make_input_row = [=](std::string_view label,
                                    text_field::internal::BasicTextField*& input,
                                    QString const& default_value) {
        auto* field = make_parameter_field(theme, font, 116, measurements, default_value);
        input       = field;

        return new Row {
            row::pro::Spacing { 8 },
            row::pro::Item<Text> {
                { 0, Qt::AlignVCenter },
                theme,
                text::pro::Font { font },
                text::pro::Text { QString::fromStdString(std::string { label }) },
                text::pro::WordWrap { true },
                text::pro::Alignment { Qt::AlignVCenter | Qt::AlignLeft },
                widget::pro::FixedWidth { 92 },
            },
            row::pro::Item { { 1, Qt::AlignVCenter }, field },
        };
    };

    convert_button = new OutlinedButton {
        theme,
        widget::pro::FixedHeight { 36 },
        widget::pro::MinimumWidth { 180 },
        button::pro::Text { "转换为点云" },
    };

    QObject::connect(convert_button, &OutlinedButton::clicked, [this](bool) {
        if (selected_asset_id.empty()) {
            return;
        }

        auto parameters            = pcs::ModelToPointcloudParameters { };
        parameters.density         = parse_double_input(density_input, 1.0, 0.01);
        parameters.sample_distance = parse_double_input(sample_distance_input, 0.0, 0.0);
        parameters.unit_scale      = parse_double_input(unit_scale_input, 1.0, 1e-6);
        parameters.max_points      = parse_size_input(max_points_input, 0, 0);

        const auto result =
            this->context.assets->convert_model_to_pointcloud(selected_asset_id, parameters);
        if (!result.has_value()) {
            spdlog::error("模型转点云失败: {}", result.error());
            return;
        }

        this->context.refresh_assets_list();
        this->context.select_asset(*result);
    });

    root = new FilledCard {
        theme,
        card::pro::LevelLow,
        card::pro::Layout<Col> {
            col::pro::Alignment { Qt::AlignTop },
            col::pro::Margin { 10 },
            col::pro::Spacing { 10 },
            col::pro::Item<Text> {
                theme,
                text::pro::Font { font },
                text::pro::Text { "模型操作" },
                text::pro::Alignment { Qt::AlignHCenter },
            },
            col::pro::Item<FilledCard> {
                theme,
                card::pro::LevelLowest,
                card::pro::Layout<Col> {
                    col::pro::Margin { 8 },
                    col::pro::Spacing { 4 },
                    col::pro::Item { make_input_row("点云密度（倍）", density_input, "1") },
                    col::pro::Item { make_input_row("采样间距（m）", sample_distance_input, "0") },
                    col::pro::Item { make_input_row("缩放比例（倍）", unit_scale_input, "1") },
                    col::pro::Item { make_input_row("最大点数（点）", max_points_input, "0") },
                },
            },
            col::pro::Item { convert_button },
        },
    };
}

auto ModelActionPanel::widget() const noexcept -> QWidget* { return root; }

auto ModelActionPanel::bind_asset(std::string const& id) noexcept -> void {
    selected_asset_id = id;
}

auto ModelActionPanel::clear() noexcept -> void { selected_asset_id.clear(); }

PngMapActionPanel::PngMapActionPanel(ActionPanelContext context, QFont const& font)
    : context { std::move(context) } {
    const auto theme = theme::pro::ThemeManager { *this->context.manager };

    save_button = new OutlinedButton {
        theme,
        widget::pro::FixedHeight { 36 },
        widget::pro::MinimumWidth { 180 },
        button::pro::Text { "保存 PNG 地图" },
    };

    QObject::connect(save_button, &OutlinedButton::clicked, [this](bool) {
        if (selected_asset_id.empty()) {
            return;
        }

        const auto suggested_name =
            this->context.assets->get_asset_name(selected_asset_id).value_or("map.png");
        if (auto location = save_png_map_location(suggested_name)) {
            const auto result =
                this->context.assets->save_png_map_asset(selected_asset_id, *location);
            if (!result.has_value()) {
                spdlog::error("保存 PNG 地图资产失败: {}", result.error());
                return;
            }

            this->context.refresh_assets_list();
            this->context.select_asset(selected_asset_id);
        }
    });

    root = new FilledCard {
        theme,
        card::pro::LevelLow,
        card::pro::Layout<Col> {
            col::pro::Alignment { Qt::AlignTop },
            col::pro::Margin { 10 },
            col::pro::Spacing { 10 },
            col::pro::Item<Text> {
                theme,
                text::pro::Font { font },
                text::pro::Text { "PNG 地图操作" },
                text::pro::Alignment { Qt::AlignHCenter },
            },
            col::pro::Item { save_button },
        },
    };
}

auto PngMapActionPanel::widget() const noexcept -> QWidget* { return root; }

auto PngMapActionPanel::bind_asset(std::string const& id) noexcept -> void {
    selected_asset_id = id;
}

auto PngMapActionPanel::clear() noexcept -> void { selected_asset_id.clear(); }

ActionPanelHost::ActionPanelHost(ActionPanelContext context, QFont const& font) {
    placeholder = new Text {
        theme::pro::ThemeManager { *context.manager },
        text::pro::Text { "暂无专属操作" },
        text::pro::Alignment { Qt::AlignHCenter },
        text::pro::Font { font },
    };

    pointcloud_panel = std::make_unique<PointcloudActionPanel>(context, font);
    model_panel      = std::make_unique<ModelActionPanel>(context, font);
    png_map_panel    = std::make_unique<PngMapActionPanel>(context, font);

    stack = new Stacked {
        stacked::pro::Item { placeholder },
        stacked::pro::Item { pointcloud_panel->widget() },
        stacked::pro::Item { model_panel->widget() },
        stacked::pro::Item { png_map_panel->widget() },
        stacked::pro::CurrentIndex { kPlaceholderIndex },
    };

    root = new Widget {
        widget::pro::Apply {
            [](QWidget& self) { self.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed); } },
        widget::pro::Layout { stack },
    };

    sync_current_panel_height();
}

auto ActionPanelHost::widget() const noexcept -> QWidget* { return root; }

auto ActionPanelHost::clear() noexcept -> void {
    pointcloud_panel->clear();
    model_panel->clear();
    png_map_panel->clear();
    stack->setCurrentIndex(kPlaceholderIndex);
    sync_current_panel_height();
}

auto ActionPanelHost::bind_asset(pcs::AssetKind kind, std::string const& id) noexcept -> void {
    switch (kind) {
    case pcs::AssetKind::Pointcloud:
        pointcloud_panel->bind_asset(id);
        stack->setCurrentIndex(kPointcloudIndex);
        sync_current_panel_height();
        return;
    case pcs::AssetKind::Model:
        model_panel->bind_asset(id);
        stack->setCurrentIndex(kModelIndex);
        sync_current_panel_height();
        return;
    case pcs::AssetKind::PngMap:
        png_map_panel->bind_asset(id);
        stack->setCurrentIndex(kPngMapIndex);
        sync_current_panel_height();
        return;
    }
}

auto ActionPanelHost::sync_current_panel_height() noexcept -> void {
    if (root == nullptr || stack == nullptr) {
        return;
    }

    auto* current = stack->currentWidget();
    if (current == nullptr) {
        return;
    }

    if (auto* layout = current->layout()) {
        layout->activate();
    }

    const auto height = std::max(current->minimumSizeHint().height(), current->sizeHint().height());
    root->setFixedHeight(height);
    root->updateGeometry();
}

}
