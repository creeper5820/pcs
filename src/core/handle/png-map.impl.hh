#pragma once

#include "core/handle/png-map.hh"
#include "core/units/png-map.hh"

#include <QImage>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>

using namespace pcs;

struct PngMapHandle::Impl final {
    PngMapData data;
    std::unique_ptr<PngMapUnit> unit;

    auto make_image(PngMapExportMirror mirror) const noexcept -> QImage {
        auto image = QImage(
            static_cast<int>(data.width), static_cast<int>(data.height), QImage::Format_Grayscale8);

        if (image.isNull()) {
            return image;
        }

        for (std::size_t y = 0; y < data.height; ++y) {
            auto* row_ptr = image.scanLine(static_cast<int>(y));
            std::memcpy(row_ptr, data.pixels.data() + y * data.width, data.width * sizeof(std::uint8_t));
        }

        switch (mirror) {
        case PngMapExportMirror::Horizontal:
            return image.flipped(Qt::Horizontal);
        case PngMapExportMirror::Vertical:
            return image.flipped(Qt::Vertical);
        case PngMapExportMirror::None:
            return image;
        }

        return image;
    }

    auto load_from_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view> {
        auto image = QImage(QString::fromStdString(path));
        if (image.isNull()) {
            return std::unexpected { "Failed to load png map from filesystem" };
        }

        auto grayscale = image.convertToFormat(QImage::Format_Grayscale8);
        if (grayscale.isNull()) {
            return std::unexpected { "Failed to convert png map into grayscale" };
        }

        auto map         = PngMapData { };
        map.width        = static_cast<std::size_t>(grayscale.width());
        map.height       = static_cast<std::size_t>(grayscale.height());
        map.resolution   = 0.1;
        map.plane_z      = 0.0;
        map.origin_x     = 0.0;
        map.origin_y     = 0.0;
        map.z_area_start = 0.0;
        map.z_area_end   = 1.0;
        map.pixels.resize(map.width * map.height);

        for (std::size_t y = 0; y < map.height; ++y) {
            auto const* row_ptr = grayscale.constScanLine(static_cast<int>(y));
            std::memcpy(
                map.pixels.data() + y * map.width, row_ptr, map.width * sizeof(std::uint8_t));
        }

        return load_from_data(map);
    }

    auto load_from_data(PngMapData const& map) noexcept -> std::expected<void, std::string_view> {
        if (map.width == 0 || map.height == 0 || map.pixels.size() != map.width * map.height) {
            return std::unexpected { "Invalid png map data" };
        }

        data = map;
        unit = std::make_unique<PngMapUnit>(data);
        return { };
    }

    auto save_into_filesystem(std::string const& path) const noexcept
        -> std::expected<void, std::string_view> {
        if (data.width == 0 || data.height == 0 || data.pixels.empty()) {
            return std::unexpected { "PNG map is not loaded" };
        }

        auto image = make_image(PngMapExportMirror::None);
        if (image.isNull()) {
            return std::unexpected { "Failed to create png image buffer" };
        }

        if (!image.save(QString::fromStdString(path), "PNG")) {
            return std::unexpected { "Failed to save png map into filesystem" };
        }

        return { };
    }

    auto export_to_ros_directory(std::string const& directory) const noexcept
        -> std::expected<void, std::string_view> {
        if (data.width == 0 || data.height == 0 || data.pixels.empty()) {
            return std::unexpected { "PNG map is not loaded" };
        }

        auto base = std::filesystem::path(directory);
        if (base.empty()) {
            return std::unexpected { "ROS export directory is empty" };
        }

        auto error = std::error_code { };
        std::filesystem::create_directories(base, error);
        if (error) {
            return std::unexpected { "Failed to create ROS export directory" };
        }

        auto image = make_image(data.frame_config.export_mirror);
        if (image.isNull()) {
            return std::unexpected { "Failed to create png image buffer" };
        }

        const auto image_path = (base / "map.png").string();
        if (!image.save(QString::fromStdString(image_path), "PNG")) {
            return std::unexpected { "Failed to save png map into filesystem" };
        }

        auto yaml = std::ofstream { base / "map.yaml", std::ios::out | std::ios::trunc };
        if (!yaml.is_open()) {
            return std::unexpected { "Failed to create ROS map yaml" };
        }

        const auto ros_origin = png_map_ros_origin(make_png_map_transform_view(data));
        yaml << std::fixed << std::setprecision(6);
        yaml << "image: map.png\n";
        yaml << "mode: trinary\n";
        yaml << "resolution: " << data.resolution << "\n";
        yaml << "origin: [" << ros_origin[0] << ", " << ros_origin[1] << ", " << ros_origin[2]
             << "]\n";
        yaml << "negate: 0\n";
        yaml << "occupied_thresh: 0.650000\n";
        yaml << "free_thresh: 0.196000\n";
        yaml.close();

        if (!yaml) {
            return std::unexpected { "Failed to write ROS map yaml" };
        }

        return { };
    }

    auto set_frame_config(PngMapFrameConfig const& config) noexcept -> bool {
        data.frame_config = config;
        if (unit == nullptr) {
            return false;
        }

        unit->update_frame_config(data);
        return true;
    }

    auto copy_pixels() const noexcept -> std::vector<std::uint8_t> { return data.pixels; }

    auto overwrite_pixels(std::vector<std::uint8_t> const& pixels) noexcept -> bool {
        if (unit == nullptr || data.width == 0 || data.height == 0) {
            return false;
        }

        if (pixels.size() != data.width * data.height) {
            return false;
        }

        data.pixels = pixels;
        return unit->update_pixels(data.pixels);
    }

    auto preview_pixels(std::vector<std::uint8_t> const& pixels) noexcept -> bool {
        if (unit == nullptr || data.width == 0 || data.height == 0) {
            return false;
        }

        if (pixels.size() != data.width * data.height) {
            return false;
        }

        return unit->update_pixels(pixels);
    }

    auto clear_preview() noexcept -> bool {
        if (unit == nullptr || data.width == 0 || data.height == 0) {
            return false;
        }

        return unit->update_pixels(data.pixels);
    }
};
