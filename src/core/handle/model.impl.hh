#pragma once

#include "core/handle/model.hh"
#include "core/units/model.hh"

#include <array>
#include <charconv>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr auto kObjToSceneScale = 0.001;

auto parse_index_token(std::string_view token, std::size_t vertex_count) noexcept
    -> std::optional<std::size_t> {
    const auto slash = token.find('/');
    token            = token.substr(0, slash);

    if (token.empty()) {
        return std::nullopt;
    }

    auto parsed = int { 0 };
    auto result = std::from_chars(token.data(), token.data() + token.size(), parsed);
    if (result.ec != std::errc { }) {
        return std::nullopt;
    }

    if (parsed > 0) {
        return static_cast<std::size_t>(parsed - 1);
    }

    if (parsed < 0) {
        const auto index = static_cast<long long>(vertex_count) + parsed;
        if (index >= 0) {
            return static_cast<std::size_t>(index);
        }
    }

    return std::nullopt;
}

auto load_obj_data(std::string const& path) noexcept
    -> std::expected<pcs::ModelData, std::string_view> {
    auto input = std::ifstream { path };
    if (!input.is_open()) {
        return std::unexpected { "Failed to open model from filesystem" };
    }

    auto model = pcs::ModelData { };

    for (auto line = std::string { }; std::getline(input, line);) {
        auto stream = std::istringstream { line };
        auto prefix = std::string { };
        stream >> prefix;

        if (prefix == "v") {
            auto x = double { };
            auto y = double { };
            auto z = double { };
            if (stream >> x >> y >> z) {
                model.vertices.push_back(
                    { x * kObjToSceneScale, y * kObjToSceneScale, z * kObjToSceneScale });
            }
            continue;
        }

        if (prefix != "f") {
            continue;
        }

        auto vertices = std::vector<std::size_t> { };
        for (auto token = std::string { }; stream >> token;) {
            auto index = parse_index_token(token, model.vertices.size());
            if (index.has_value()) {
                vertices.push_back(*index);
            }
        }

        if (vertices.size() < 3) {
            continue;
        }

        for (std::size_t i = 1; i + 1 < vertices.size(); ++i) {
            model.faces.push_back({ vertices[0], vertices[i], vertices[i + 1] });
        }
    }

    if (model.vertices.empty()) {
        return std::unexpected { "Failed to read model from filesystem" };
    }

    return model;
}

}

using namespace pcs;

struct ModelHandle::Impl final {
    std::unique_ptr<ModelUnit> unit;
    ModelData data;

    auto load_from_filesystem(std::string const& path) noexcept
        -> std::expected<void, std::string_view> {
        auto result = load_obj_data(path);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        data = std::move(result).value();
        unit = std::make_unique<ModelUnit>(data);
        return { };
    }
};
