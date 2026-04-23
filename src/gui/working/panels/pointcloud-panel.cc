#include "gui/working/panels/pointcloud-panel.hh"

#include "core/events/process/pointcloud-assets.hh"
#include "core/events/process/pointcloud-cluster-filter.hh"
#include "core/events/process/pointcloud-range-crop.hh"
#include "core/events/process/pointcloud-to-png-map.hh"
#include "core/events/process/pointcloud-transform.hh"
#include "core/handle/crop-box.hh"
#include "core/handle/png-map.hh"
#include "core/handle/points.hh"
#include "gui/component/expanded-field-row.hh"
#include "gui/working/panels/common.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/utility/wrapper/mutable-value.hh>
#include <creeper-qt/widget/buttons/outlined-button.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/sliders.hh>
#include <creeper-qt/widget/switch.hh>
#include <creeper-qt/widget/text-fields.hh>
#include <creeper-qt/widget/text.hh>
#include <creeper-qt/widget/widget.hh>

#include <QSignalBlocker>
#include <QTimer>

#include <qmessagebox.h>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <tuple>
#include <unordered_map>
#include <vector>

using namespace creeper;
using namespace pcs::gui::component;

namespace pcs::gui::working {

namespace {

    constexpr auto kLargePointcloudBytes       = std::uintmax_t { 100 } * 1024U * 1024U;
    constexpr auto kLargePointcloudRangeMeters = 100.0;

    struct Bounds3D {
        double x_min = 0.0;
        double x_max = 0.0;
        double y_min = 0.0;
        double y_max = 0.0;
        double z_min = 0.0;
        double z_max = 0.0;
    };

    auto compute_bounds_from_points(
        std::vector<pcs::event::ConvertPointcloudToPngMap::Position> const& points) noexcept
        -> std::optional<Bounds3D> {
        if (points.empty()) {
            return std::nullopt;
        }

        auto bounds = Bounds3D {
            .x_min = std::get<0>(points.front()),
            .x_max = std::get<0>(points.front()),
            .y_min = std::get<1>(points.front()),
            .y_max = std::get<1>(points.front()),
            .z_min = std::get<2>(points.front()),
            .z_max = std::get<2>(points.front()),
        };

        for (auto const& point : points) {
            const auto x = std::get<0>(point);
            const auto y = std::get<1>(point);
            const auto z = std::get<2>(point);
            bounds.x_min = std::min(bounds.x_min, x);
            bounds.x_max = std::max(bounds.x_max, x);
            bounds.y_min = std::min(bounds.y_min, y);
            bounds.y_max = std::max(bounds.y_max, y);
            bounds.z_min = std::min(bounds.z_min, z);
            bounds.z_max = std::max(bounds.z_max, z);
        }

        return bounds;
    }

    auto confirm_large_pointcloud_extent_warning(Bounds3D const& bounds) noexcept -> bool {
        const auto x_range   = bounds.x_max - bounds.x_min;
        const auto y_range   = bounds.y_max - bounds.y_min;
        const auto z_range   = bounds.z_max - bounds.z_min;
        const auto max_range = std::max({ x_range, y_range, z_range });

        if (max_range <= kLargePointcloudRangeMeters) {
            return true;
        }

        const auto message = QString("当前点云范围较大：X=%1 m，Y=%2 m，Z=%3 m，最大范围为 %4 "
                                     "m（超过 100 m），生成 PNG "
                                     "地图可能异常或占用大量内存，是否继续？")
                                 .arg(x_range, 0, 'f', 2)
                                 .arg(y_range, 0, 'f', 2)
                                 .arg(z_range, 0, 'f', 2)
                                 .arg(max_range, 0, 'f', 2);

        const auto answer = QMessageBox::warning(
            nullptr, "点云范围提示", message, QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

        return answer == QMessageBox::Yes;
    }

    class AlgorithmExecutionGroup final {
    public:
        auto add(OutlinedButton* button, QString idle_text, QString busy_text) noexcept
            -> void {
            entries.push_back({ button, std::move(idle_text), std::move(busy_text) });
            sync_buttons();
        }

        auto begin(OutlinedButton* button) noexcept -> bool {
            if (running_button != nullptr) {
                return false;
            }

            running_button = button;
            sync_buttons();
            return true;
        }

        auto end(OutlinedButton* button) noexcept -> void {
            if (running_button != button) {
                return;
            }

            running_button = nullptr;
            sync_buttons();
        }

    private:
        struct Entry {
            OutlinedButton* button = nullptr;
            QString idle_text;
            QString busy_text;
        };

        auto sync_buttons() noexcept -> void {
            for (auto const& entry : entries) {
                if (running_button == nullptr) {
                    entry.button->setDisabled(false);
                    entry.button->setText(entry.idle_text);
                    continue;
                }

                entry.button->setDisabled(entry.button != running_button);
                entry.button->setText(
                    running_button == entry.button ? entry.busy_text : entry.idle_text);
            }
        }

        std::vector<Entry> entries;
        OutlinedButton* running_button = nullptr;
    };

    class PointcloudPanelBody final : public AssetActionPanel {
    public:
        explicit PointcloudPanelBody(ActionPanelContext context, QFont const& font)
            : context { std::move(context) }
            , assets { *this->context.assets }
            , runtime { *this->context.runtime }
            , renderer { *this->context.renderer }
            , mouse { *this->context.mouse } {
            using namespace creeper;
            namespace ob     = outlined_button::pro;
            namespace bp     = button::pro;
            namespace ev     = pcs::event;
            const auto theme = ob::ThemeManager { *this->context.manager };

            crop_box.attach_renderer(renderer);
            crop_box.set_visibility(false);

            const auto update_color = [this] {
                if (selected_handle == nullptr) {
                    return;
                }

                selected_handle->set_overall_color(
                    *color_channels[0], *color_channels[1], *color_channels[2], *color_channels[3]);
                assets.update_renderer();
            };

            color_rows[0] =
                new panels::ValueSliderRow(theme, font, "R", color_channels[0], update_color);
            color_rows[1] =
                new panels::ValueSliderRow(theme, font, "G", color_channels[1], update_color);
            color_rows[2] =
                new panels::ValueSliderRow(theme, font, "B", color_channels[2], update_color);
            color_rows[3] =
                new panels::ValueSliderRow(theme, font, "A", color_channels[3], update_color);

            resolution_row =
                new panels::CompactFieldRow(theme, font, "分辨率（m）", 92, 116, "0.1");
            resolution_input = &resolution_row->field();

            points_limit_row =
                new panels::CompactFieldRow(theme, font, "有效点云数（点）", 92, 116, "5");
            points_limit_input = &points_limit_row->field();

            height_limit_row =
                new panels::CompactFieldRow(theme, font, "有效高度差（m）", 92, 116, "0.1");
            height_limit_input = &height_limit_row->field();

            influence_radius_row =
                new panels::CompactFieldRow(theme, font, "影响半径（m）", 92, 116, "0.08");
            influence_radius_input = &influence_radius_row->field();

            z_area_row =
                new panels::CompactDualFieldRow(theme, font, "Z 区间（m）", 92, 56, "0", "1");
            z_area_start_input = &z_area_row->first();
            z_area_end_input   = &z_area_row->second();

            cluster_tolerance_row =
                new panels::CompactFieldRow(theme, font, "聚类距离（m）", 92, 116, "0.3");
            cluster_tolerance_input = &cluster_tolerance_row->field();

            cluster_min_cluster_row =
                new panels::CompactFieldRow(theme, font, "最小簇点数", 92, 116, "20");
            cluster_min_cluster_input = &cluster_min_cluster_row->field();

            range_x_row = new AxisRangeRow(theme, font, "X", "", "", "-X", "+X");
            range_y_row = new AxisRangeRow(theme, font, "Y", "", "", "-Y", "+Y");
            range_z_row = new AxisRangeRow(theme, font, "Z", "", "", "-Z", "+Z");

            translate_row = new ExpandedTripleFieldRow(theme, font, "", "", "", "X", "Y", "Z");

            rotate_row =
                new ExpandedTripleFieldRow(theme, font, "", "", "", "Yaw", "Pitch", "Roll");

            pivot_xyz_row = new ExpandedTripleFieldRow(theme, font, "", "", "", "X", "Y", "Z");

            namespace sw      = _switch::pro;
            coordinate_switch = new Switch {
                sw::ThemeManager { *this->context.manager },
                sw::Checked { true },
                sw::FixedSize { 44, 24 },
                sw::Clickable { [this] {
                    if (selected_handle == nullptr) {
                        return;
                    }

                    const auto next_visibility = !selected_handle->coordinate_visibility();
                    selected_handle->set_coordinate_visibility(next_visibility);
                    const auto blocker = QSignalBlocker { coordinate_switch };
                    coordinate_switch->set_checked(next_visibility);
                    assets.update_renderer();
                } },
            };
            coordinate_switch_row =
                new panels::CompactWidgetRow(theme, font, "显示坐标系", 92, coordinate_switch);

            const auto current_bounds = [this]() -> std::optional<Bounds3D> {
                return compute_selected_asset_bounds();
            };

            const auto sync_crop_preview = [this]() -> bool {
                return sync_crop_preview_for_selected_asset();
            };

            const auto bind_crop_input_refresh = [this, sync_crop_preview](
                                                     OutlinedTextField& input) {
                QObject::connect(
                    &input, &OutlinedTextField::editingFinished, [this, sync_crop_preview]() {
                        if (!crop_area_visible) {
                            return;
                        }
                        (void)sync_crop_preview();
                    });
            };

            bind_crop_input_refresh(range_x_row->first());
            bind_crop_input_refresh(range_x_row->second());
            bind_crop_input_refresh(range_y_row->first());
            bind_crop_input_refresh(range_y_row->second());
            bind_crop_input_refresh(range_z_row->first());
            bind_crop_input_refresh(range_z_row->second());

            const auto set_generate_busy = [this](bool busy) {
                generate_button->setDisabled(busy);
                generate_button->setText(busy ? QString::fromUtf8("生成中") : "生成 PNG 地图");
            };

            const auto should_continue_large_generation = [this](std::size_t points_count) -> bool {
                const auto path = assets.get_asset_path(selected_asset_id);
                if (!path.empty() && path != "<memory>") {
                    auto error      = std::error_code { };
                    const auto size = std::filesystem::file_size(path, error);
                    if (!error && size > kLargePointcloudBytes) {
                        return panels::confirm_large_pointcloud_warning(size);
                    }
                    return true;
                }

                constexpr auto bytes_per_point = sizeof(double) * 3U;
                const auto estimated_size =
                    static_cast<std::uintmax_t>(points_count) * bytes_per_point;
                if (estimated_size > kLargePointcloudBytes) {
                    return panels::confirm_large_pointcloud_warning(estimated_size);
                }

                return true;
            };

            generate_button = new OutlinedButton {
                theme,
                ob::FixedHeight { 36 },
                ob::MinimumWidth { 180 },
                ob::Text { "生成 PNG 地图" },
                bp::Clickable { [this, set_generate_busy, should_continue_large_generation] {
                    if (selected_asset_id.empty() || selected_handle == nullptr) {
                        return;
                    }
                    if (!generate_button->isEnabled()) {
                        return;
                    }

                    auto source_points = selected_handle->get_positions();
                    if (source_points.empty()) {
                        spdlog::error("点云为空");
                        return;
                    }

                    const auto bounds = compute_bounds_from_points(source_points);
                    if (!bounds.has_value()) {
                        spdlog::error("点云范围不可用");
                        return;
                    }

                    if (!confirm_large_pointcloud_extent_warning(*bounds)) {
                        return;
                    }

                    if (!should_continue_large_generation(source_points.size())) {
                        return;
                    }

                    auto params         = pcs::PngMapParameters { };
                    params.resolution   = panels::parse_double_input(*resolution_input, 0.1, 0.01);
                    params.points_limit = panels::parse_size_input(*points_limit_input, 5, 1);
                    params.height_limit = panels::parse_double_input(*height_limit_input, 0.1, 0.0);
                    params.influence_radius =
                        panels::parse_double_input(*influence_radius_input, 0.08, 0.0);
                    params.z_area_start = panels::parse_double_input(*z_area_start_input, 0.0, 0.0);
                    params.z_area_end   = panels::parse_double_input(*z_area_end_input, 1.0, 0.0);

                    set_generate_busy(true);

                    auto source_asset_id = std::string { selected_asset_id };

                    auto event       = ev::ConvertPointcloudToPngMap { };
                    event.points     = std::move(source_points);
                    event.parameters = params;

                    auto future =
                        std::make_shared<std::future<ev::ConvertPointcloudToPngMap::Result>>(
                            runtime.submit(std::move(event)));

                    auto* watcher = new QTimer { generate_button };
                    watcher->setInterval(30);
                    QObject::connect(watcher, &QTimer::timeout,
                        [this, watcher, source_asset_id = std::move(source_asset_id), future,
                            set_generate_busy]() mutable {
                            if (future->wait_for(std::chrono::seconds { 0 })
                                != std::future_status::ready) {
                                return;
                            }

                            watcher->stop();
                            watcher->deleteLater();
                            set_generate_busy(false);

                            auto compute_result = future->get();
                            if (!compute_result.has_value()) {
                                spdlog::error("点云生成 PNG 地图失败: {}", compute_result.error());
                                return;
                            }

                            auto png_map     = std::make_unique<PngMapHandle>();
                            auto load_result = png_map->load_from_data(compute_result.value());
                            if (!load_result.has_value()) {
                                spdlog::error("创建 PNG 地图资产失败: {}", load_result.error());
                                return;
                            }

                            auto source_name = assets.get_asset_name(source_asset_id);
                            auto derived     = std::filesystem::path(source_name);
                            if (derived.empty()) {
                                derived = "generated-map.png";
                            } else {
                                derived.replace_extension(".png");
                            }

                            if (auto iter = generated_png_asset_ids.find(source_asset_id);
                                iter != generated_png_asset_ids.end()) {
                                const auto& previous_generated_id = iter->second;
                                if (auto existing_generated =
                                        assets.get_handle<PngMapHandle>(previous_generated_id);
                                    existing_generated.has_value()) {
                                    (void)assets.remove_asset(previous_generated_id);
                                }
                            }

                            auto generated_id = assets.register_asset<PngMapHandle>(
                                std::move(png_map), derived.filename().string(), { }, false);
                            generated_png_asset_ids[source_asset_id] = generated_id;

                            this->context.refresh_assets_list();
                            this->context.select_asset(source_asset_id);
                        });
                    watcher->start();
                } },
            };

            cluster_keep_largest_button = new OutlinedButton {
                theme,
                ob::FixedHeight { 34 },
                ob::Text { "保留最大簇" },
                bp::Clickable { [this] {
                    if (selected_asset_id.empty() || selected_handle == nullptr) {
                        return;
                    }

                    auto before_points = selected_handle->get_positions();
                    auto event         = ev::PointcloudClusterFilter { };
                    event.points       = before_points;
                    event.tolerance =
                        panels::parse_double_input(*cluster_tolerance_input, 0.3, 1e-6);
                    event.min_cluster_size =
                        panels::parse_size_input(*cluster_min_cluster_input, 20, 1);
                    event.mode = ev::PointcloudClusterMode::KeepLargestCluster;

                    auto future =
                        std::make_shared<std::future<ev::PointcloudClusterFilter::Result>>(
                            runtime.submit(std::move(event)));
                    if (!algorithm_execution_group.begin(cluster_keep_largest_button)) {
                        return;
                    }

                    auto* watcher = new QTimer { cluster_keep_largest_button };
                    watcher->setInterval(30);
                    QObject::connect(watcher, &QTimer::timeout,
                        [this, watcher, future,
                            before_points_snapshot = std::move(before_points)]() mutable {
                            if (future->wait_for(std::chrono::seconds { 0 })
                                != std::future_status::ready) {
                                return;
                            }

                            watcher->stop();
                            watcher->deleteLater();
                            algorithm_execution_group.end(cluster_keep_largest_button);

                            auto result = future->get();
                            if (!result.has_value()) {
                                spdlog::error("点云聚类失败: {}", result.error());
                                return;
                            }

                            auto apply_result = apply_points_with_history(
                                before_points_snapshot, std::move(result.value()));
                            if (!apply_result.has_value()) {
                                spdlog::error("应用聚类结果失败: {}", apply_result.error());
                            }
                        });
                    watcher->start();
                } },
            };
            cluster_remove_small_button = new OutlinedButton {
                theme,
                ob::FixedHeight { 34 },
                ob::Text { "去除较小簇" },
                bp::Clickable { [this] {
                    if (selected_asset_id.empty() || selected_handle == nullptr) {
                        return;
                    }

                    auto before_points = selected_handle->get_positions();
                    auto event         = ev::PointcloudClusterFilter { };
                    event.points       = before_points;
                    event.tolerance =
                        panels::parse_double_input(*cluster_tolerance_input, 0.3, 1e-6);
                    event.min_cluster_size =
                        panels::parse_size_input(*cluster_min_cluster_input, 20, 1);
                    event.mode = ev::PointcloudClusterMode::RemoveSmallClusters;

                    auto future =
                        std::make_shared<std::future<ev::PointcloudClusterFilter::Result>>(
                            runtime.submit(std::move(event)));
                    if (!algorithm_execution_group.begin(cluster_remove_small_button)) {
                        return;
                    }

                    auto* watcher = new QTimer { cluster_remove_small_button };
                    watcher->setInterval(30);
                    QObject::connect(watcher, &QTimer::timeout,
                        [this, watcher, future,
                            before_points_snapshot = std::move(before_points)]() mutable {
                            if (future->wait_for(std::chrono::seconds { 0 })
                                != std::future_status::ready) {
                                return;
                            }

                            watcher->stop();
                            watcher->deleteLater();
                            algorithm_execution_group.end(cluster_remove_small_button);

                            auto result = future->get();
                            if (!result.has_value()) {
                                spdlog::error("点云聚类失败: {}", result.error());
                                return;
                            }

                            auto apply_result = apply_points_with_history(
                                before_points_snapshot, std::move(result.value()));
                            if (!apply_result.has_value()) {
                                spdlog::error("应用聚类结果失败: {}", apply_result.error());
                            }
                        });
                    watcher->start();
                } },
            };
            range_crop_button = new OutlinedButton {
                theme,
                ob::FixedHeight { 34 },
                ob::Text { "按范围截取" },
                bp::Clickable { [this, current_bounds, sync_crop_preview] {
                    if (selected_asset_id.empty() || selected_handle == nullptr) {
                        return;
                    }

                    auto before_points = selected_handle->get_positions();
                    auto bounds        = current_bounds();
                    if (!bounds.has_value()) {
                        spdlog::error("范围截取失败: 点云范围不可用");
                        return;
                    }

                    auto event   = ev::PointcloudRangeCrop { };
                    event.points = before_points;
                    event.x_min  = range_x_row->parse_first_double_keep_empty(bounds->x_min, -1e12);
                    event.x_max = range_x_row->parse_second_double_keep_empty(bounds->x_max, -1e12);
                    event.y_min = range_y_row->parse_first_double_keep_empty(bounds->y_min, -1e12);
                    event.y_max = range_y_row->parse_second_double_keep_empty(bounds->y_max, -1e12);
                    event.z_min = range_z_row->parse_first_double_keep_empty(bounds->z_min, -1e12);
                    event.z_max = range_z_row->parse_second_double_keep_empty(bounds->z_max, -1e12);

                    auto future = std::make_shared<std::future<ev::PointcloudRangeCrop::Result>>(
                        runtime.submit(std::move(event)));
                    if (!algorithm_execution_group.begin(range_crop_button)) {
                        return;
                    }

                    auto* watcher = new QTimer { range_crop_button };
                    watcher->setInterval(30);
                    QObject::connect(watcher, &QTimer::timeout,
                        [this, watcher, future, before_points_snapshot = std::move(before_points),
                            sync_crop_preview]() mutable {
                            if (future->wait_for(std::chrono::seconds { 0 })
                                != std::future_status::ready) {
                                return;
                            }

                            watcher->stop();
                            watcher->deleteLater();
                            algorithm_execution_group.end(range_crop_button);

                            auto result = future->get();
                            if (!result.has_value()) {
                                spdlog::error("范围截取失败: {}", result.error());
                                return;
                            }

                            auto apply_result = apply_points_with_history(
                                before_points_snapshot, std::move(result.value()));
                            if (!apply_result.has_value()) {
                                spdlog::error("应用范围截取失败: {}", apply_result.error());
                                return;
                            }

                            if (crop_area_visible) {
                                (void)sync_crop_preview();
                            }
                        });
                    watcher->start();
                } },
            };
            range_area_toggle_button = new OutlinedButton {
                theme,
                ob::FixedHeight { 34 },
                ob::Text { "显示截取区域" },
                bp::Clickable { [this, sync_crop_preview] {
                    crop_area_visible = !crop_area_visible;
                    range_area_toggle_button->setText(
                        crop_area_visible ? "隐藏截取区域" : "显示截取区域");

                    if (crop_area_visible) {
                        (void)sync_crop_preview();
                    } else {
                        crop_box.set_visibility(false);
                        assets.update_renderer();
                    }

                    this->context.mouse->set_status(
                        crop_area_visible ? "截取区域显示" : "截取区域隐藏");
                } },
            };
            transform_button = new OutlinedButton {
                theme,
                ob::FixedHeight { 34 },
                ob::Text { "平移旋转" },
                bp::Clickable { [this] {
                    if (selected_asset_id.empty() || selected_handle == nullptr) {
                        return;
                    }

                    auto before_points = selected_handle->get_positions();
                    auto event         = ev::PointcloudTransform { };
                    event.points       = before_points;
                    event.tx           = translate_row->parse_first_double_keep_empty(0.0, -1e9);
                    event.ty           = translate_row->parse_second_double_keep_empty(0.0, -1e9);
                    event.tz           = translate_row->parse_third_double_keep_empty(0.0, -1e9);
                    event.yaw_deg      = rotate_row->parse_first_double_keep_empty(0.0, -1e9);
                    event.pitch_deg    = rotate_row->parse_second_double_keep_empty(0.0, -1e9);
                    event.roll_deg     = rotate_row->parse_third_double_keep_empty(0.0, -1e9);
                    event.pivot_x      = pivot_xyz_row->parse_first_double_keep_empty(0.0, -1e9);
                    event.pivot_y      = pivot_xyz_row->parse_second_double_keep_empty(0.0, -1e9);
                    event.pivot_z      = pivot_xyz_row->parse_third_double_keep_empty(0.0, -1e9);

                    auto future = std::make_shared<std::future<ev::PointcloudTransform::Result>>(
                        runtime.submit(std::move(event)));
                    if (!algorithm_execution_group.begin(transform_button)) {
                        return;
                    }

                    auto* watcher = new QTimer { transform_button };
                    watcher->setInterval(30);
                    QObject::connect(watcher, &QTimer::timeout,
                        [this, watcher, future,
                            before_points_snapshot = std::move(before_points)]() mutable {
                            if (future->wait_for(std::chrono::seconds { 0 })
                                != std::future_status::ready) {
                                return;
                            }

                            watcher->stop();
                            watcher->deleteLater();
                            algorithm_execution_group.end(transform_button);

                            auto result = future->get();
                            if (!result.has_value()) {
                                spdlog::error("平移旋转失败: {}", result.error());
                                return;
                            }

                            auto apply_result = apply_points_with_history(
                                before_points_snapshot, std::move(result.value()));
                            if (!apply_result.has_value()) {
                                spdlog::error("应用平移旋转失败: {}", apply_result.error());
                            }
                        });
                    watcher->start();
                } },
            };
            algorithm_execution_group.add(cluster_keep_largest_button, "保留最大簇", "处理中...");
            algorithm_execution_group.add(cluster_remove_small_button, "去除较小簇", "处理中...");
            algorithm_execution_group.add(range_crop_button, "按范围截取", "处理中...");
            algorithm_execution_group.add(transform_button, "平移旋转", "处理中...");

            auto process_title_font = font;
            process_title_font.setPointSize(font.pointSize() + 1);

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
                            col::pro::Margin { 24 },
                            col::pro::Spacing { 4 },
                            col::pro::Item { coordinate_switch_row },
                            col::pro::Item { color_rows[0] },
                            col::pro::Item { color_rows[1] },
                            col::pro::Item { color_rows[2] },
                            col::pro::Item { color_rows[3] },
                        },
                    },
                    col::pro::Item<FilledCard> {
                        theme,
                        card::pro::LevelLowest,
                        card::pro::Layout<Col> {
                            col::pro::Margin { 24 },
                            col::pro::Spacing { 4 },
                            col::pro::Item { resolution_row },
                            col::pro::Item { points_limit_row },
                            col::pro::Item { height_limit_row },
                            col::pro::Item { influence_radius_row },
                            col::pro::Item { z_area_row },
                            col::pro::Item { generate_button },
                        },
                    },
                    col::pro::Item<FilledCard> {
                        theme,
                        card::pro::LevelLowest,
                        card::pro::Layout<Col> {
                            col::pro::Margin { 24 },
                            col::pro::Spacing { 4 },
                            col::pro::Item<Text> {
                                theme,
                                text::pro::Font { process_title_font },
                                text::pro::Text { "点云处理" },
                                text::pro::Alignment { Qt::AlignHCenter },
                            },
                            col::pro::Item { cluster_tolerance_row },
                            col::pro::Item { cluster_min_cluster_row },
                            col::pro::Item<Row> {
                                row::pro::Spacing { 6 },
                                row::pro::Item { { 1 }, cluster_keep_largest_button },
                                row::pro::Item { { 1 }, cluster_remove_small_button },
                            },
                            col::pro::Item<Text> {
                                theme,
                                text::pro::Font { process_title_font },
                                text::pro::Text { "截取" },
                                text::pro::Alignment { Qt::AlignHCenter },
                            },
                            col::pro::Item { range_x_row },
                            col::pro::Item { range_y_row },
                            col::pro::Item { range_z_row },
                            col::pro::Item<Row> {
                                row::pro::Spacing { 6 },
                                row::pro::Item { { 1 }, range_crop_button },
                                row::pro::Item { { 1 }, range_area_toggle_button },
                            },
                            col::pro::Item<Text> {
                                theme,
                                text::pro::Font { process_title_font },
                                text::pro::Text { "平移" },
                                text::pro::Alignment { Qt::AlignHCenter },
                            },
                            col::pro::Item { translate_row },
                            col::pro::Item<Text> {
                                theme,
                                text::pro::Font { process_title_font },
                                text::pro::Text { "旋转" },
                                text::pro::Alignment { Qt::AlignHCenter },
                            },
                            col::pro::Item { rotate_row },
                            col::pro::Item<Text> {
                                theme,
                                text::pro::Font { process_title_font },
                                text::pro::Text { "Pivot" },
                                text::pro::Alignment { Qt::AlignHCenter },
                            },
                            col::pro::Item { pivot_xyz_row },
                            col::pro::Item { transform_button },
                        },
                    },
                },
            };
        }

        auto widget() const noexcept -> QWidget* override { return root; }

        ~PointcloudPanelBody() override { crop_box.detach_renderer(renderer); }

        auto bind_asset(std::string const& id) noexcept -> void override {
            selected_asset_id = id;
            auto handle       = assets.get_handle<PointsHandle>(selected_asset_id);
            selected_handle   = handle.has_value() ? handle.value() : nullptr;
            if (selected_handle == nullptr) {
                return;
            }

            std::tie(*color_channels[0], *color_channels[1], *color_channels[2],
                *color_channels[3]) = selected_handle->get_overall_color();

            const auto blocker = QSignalBlocker { coordinate_switch };
            coordinate_switch->set_checked(selected_handle->coordinate_visibility());
            range_area_toggle_button->setText(crop_area_visible ? "隐藏截取区域" : "显示截取区域");

            if (crop_area_visible) {
                if (pending_crop_sync_from_apply) {
                    pending_crop_sync_from_apply = false;
                } else {
                    (void)sync_crop_preview_for_selected_asset();
                }
            } else {
                crop_box.set_visibility(false);
                assets.update_renderer();
            }
        }

        auto clear() noexcept -> void override {
            selected_asset_id.clear();
            selected_handle = nullptr;
            crop_box.set_visibility(false);
            assets.update_renderer();
        }

    private:
        auto compute_selected_asset_bounds() const noexcept -> std::optional<Bounds3D> {
            if (selected_handle == nullptr) {
                return std::nullopt;
            }

            return compute_bounds_from_points(selected_handle->get_positions());
        }

        auto sync_crop_preview_for_selected_asset() noexcept -> bool {
            if (!crop_area_visible) {
                return false;
            }

            const auto bounds = compute_selected_asset_bounds();
            if (!bounds.has_value()) {
                crop_box.set_visibility(false);
                assets.update_renderer();
                return false;
            }

            const auto x_min = range_x_row->parse_first_double_keep_empty(bounds->x_min, -1e12);
            const auto x_max = range_x_row->parse_second_double_keep_empty(bounds->x_max, -1e12);
            const auto y_min = range_y_row->parse_first_double_keep_empty(bounds->y_min, -1e12);
            const auto y_max = range_y_row->parse_second_double_keep_empty(bounds->y_max, -1e12);
            const auto z_min = range_z_row->parse_first_double_keep_empty(bounds->z_min, -1e12);
            const auto z_max = range_z_row->parse_second_double_keep_empty(bounds->z_max, -1e12);

            crop_box.set_bounds(x_min, x_max, y_min, y_max, z_min, z_max);
            crop_box.set_visibility(true);
            assets.update_renderer();
            return true;
        }

        auto apply_points_with_history(std::vector<pcs::event::PointcloudPosition> before,
            std::vector<pcs::event::PointcloudPosition> after) noexcept
            -> std::expected<void, std::string> {
            auto apply          = pcs::event::ApplyPointcloudAssetPoints { };
            apply.assets        = &assets;
            apply.asset_id      = selected_asset_id;
            apply.before_points = std::move(before);
            apply.after_points  = std::move(after);

            auto result = runtime.submit(std::move(apply)).get();
            if (!result.has_value()) {
                return std::unexpected { result.error() };
            }

            pending_crop_sync_from_apply = crop_area_visible;

            if (pending_crop_sync_from_apply) {
                (void)sync_crop_preview_for_selected_asset();
                pending_crop_sync_from_apply = false;
            }
            return { };
        }

        ActionPanelContext context;
        pcs::AssetsManager& assets;
        pcs::Runtime& runtime;
        pcs::Renderer& renderer;
        pcs::gui::interaction::Mouse& mouse;
        std::string selected_asset_id;
        PointsHandle* selected_handle = nullptr;

        std::array<std::shared_ptr<MutableDouble>, 4> color_channels {
            std::make_shared<MutableDouble>(),
            std::make_shared<MutableDouble>(),
            std::make_shared<MutableDouble>(),
            std::make_shared<MutableDouble>(),
        };

        std::array<panels::ValueSliderRow*, 4> color_rows { nullptr, nullptr, nullptr, nullptr };

        panels::CompactFieldRow* resolution_row       = nullptr;
        panels::CompactFieldRow* points_limit_row     = nullptr;
        panels::CompactFieldRow* height_limit_row     = nullptr;
        panels::CompactFieldRow* influence_radius_row = nullptr;
        panels::CompactDualFieldRow* z_area_row       = nullptr;

        panels::CompactFieldRow* cluster_tolerance_row   = nullptr;
        panels::CompactFieldRow* cluster_min_cluster_row = nullptr;

        AxisRangeRow* range_x_row = nullptr;
        AxisRangeRow* range_y_row = nullptr;
        AxisRangeRow* range_z_row = nullptr;

        ExpandedTripleFieldRow* translate_row = nullptr;
        ExpandedTripleFieldRow* rotate_row    = nullptr;
        ExpandedTripleFieldRow* pivot_xyz_row = nullptr;

        panels::CompactWidgetRow* coordinate_switch_row = nullptr;

        OutlinedTextField* resolution_input       = nullptr;
        OutlinedTextField* points_limit_input     = nullptr;
        OutlinedTextField* height_limit_input     = nullptr;
        OutlinedTextField* influence_radius_input = nullptr;
        OutlinedTextField* z_area_start_input     = nullptr;
        OutlinedTextField* z_area_end_input       = nullptr;

        OutlinedTextField* cluster_tolerance_input   = nullptr;
        OutlinedTextField* cluster_min_cluster_input = nullptr;

        OutlinedButton* generate_button             = nullptr;
        OutlinedButton* cluster_keep_largest_button = nullptr;
        OutlinedButton* cluster_remove_small_button = nullptr;
        OutlinedButton* range_crop_button           = nullptr;
        OutlinedButton* range_area_toggle_button    = nullptr;
        OutlinedButton* transform_button            = nullptr;
        Switch* coordinate_switch                   = nullptr;

        AlgorithmExecutionGroup algorithm_execution_group;
        pcs::CropBoxHandle crop_box;
        bool crop_area_visible            = false;
        bool pending_crop_sync_from_apply = false;
        std::unordered_map<std::string, std::string> generated_png_asset_ids;
        FilledCard* root         = nullptr;
    };

}

struct PointcloudPanel::Impl {
    std::unique_ptr<AssetActionPanel> panel;
};

PointcloudPanel::~PointcloudPanel() noexcept = default;

auto PointcloudPanel::widget() const noexcept -> QWidget* { return pimpl->panel->widget(); }

auto PointcloudPanel::bind_asset(std::string const& id) noexcept -> void {
    pimpl->panel->bind_asset(id);
}

auto PointcloudPanel::clear() noexcept -> void { pimpl->panel->clear(); }

PointcloudPanel::PointcloudPanel(ActionPanelContext context, QFont const& font) noexcept
    : pimpl { std::make_unique<Impl>() } {
    pimpl->panel = std::make_unique<PointcloudPanelBody>(std::move(context), font);
}

}
