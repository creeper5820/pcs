#pragma once

#include "core/handle/png-map.hh"
#include "core/units/png-map.hh"

#include <QImage>

#include <cstring>

using namespace pcs;

struct PngMapHandle::Impl final {
    PngMapData data;
    std::unique_ptr<PngMapUnit> unit;

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
};
