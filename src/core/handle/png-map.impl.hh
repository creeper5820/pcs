#pragma once

#include "core/handle/png-map.hh"
#include "core/units/png-map.hh"

#include <QImage>

#include <cstring>

using namespace pcs;

struct PngMapHandle::Impl final {
    PngMapData data;
    std::unique_ptr<PngMapUnit> unit;

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

        auto map       = PngMapData { };
        map.width      = static_cast<std::size_t>(grayscale.width());
        map.height     = static_cast<std::size_t>(grayscale.height());
        map.resolution = 0.1;
        map.plane_z    = 0.0;
        map.origin_x   = 0.0;
        map.origin_y   = 0.0;
        map.z_area_start = 0.0;
        map.z_area_end   = 1.0;
        map.pixels.resize(map.width * map.height);

        for (std::size_t y = 0; y < map.height; ++y) {
            auto const* row_ptr = grayscale.constScanLine(static_cast<int>(y));
            std::memcpy(map.pixels.data() + y * map.width, row_ptr, map.width * sizeof(std::uint8_t));
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

        auto image = QImage(
            static_cast<int>(data.width), static_cast<int>(data.height), QImage::Format_Grayscale8);

        if (image.isNull()) {
            return std::unexpected { "Failed to create png image buffer" };
        }

        for (std::size_t y = 0; y < data.height; ++y) {
            auto* row_ptr = image.scanLine(static_cast<int>(y));
            std::memcpy(
                row_ptr, data.pixels.data() + y * data.width, data.width * sizeof(std::uint8_t));
        }

        if (!image.save(QString::fromStdString(path), "PNG")) {
            return std::unexpected { "Failed to save png map into filesystem" };
        }

        return { };
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
