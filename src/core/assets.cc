#include "assets.hh"

#include "core/handle/points.hh"

#include <generator>
#include <spdlog/spdlog.h>

using namespace pcs;

struct AssetsManager::Impl {
    Renderer& renderer;

    struct PointsAsset {
        std::unique_ptr<PointsHandle> unit;
        bool use_default = true;

        auto release_unit(Renderer& renderer) noexcept {
            // 释放渲染单元
            unit->detach_renderer(renderer);
        }
    };

    std::tuple<double, double, double> default_point_color;

    std::unordered_map<std::string, std::unique_ptr<PointsAsset>> pointclouds;

    auto get_pointcloud_locations() const noexcept -> std::generator<std::string_view> {
        for (const auto& [key, _] : pointclouds) {
            co_yield std::string_view { key };
        }
    }

    auto clean_pointclouds() noexcept {
        for (auto& [_, asset] : pointclouds) {
            asset->release_unit(renderer);
        }
        pointclouds.clear();
        renderer.render_window();
    }
    auto set_pointclouds_visibility(bool on) noexcept -> void {
        for (auto& [_, asset] : pointclouds) {
            asset->unit->set_visibility(on);
        }
        renderer.render_window();
    }

    auto open_pointcloud_file(std::string const& location) noexcept {

        auto [r, g, b] = default_point_color;

        auto points = std::make_unique<PointsHandle>();
        auto result = points->load_from_filesystem(location);

        if (!result.has_value()) {
            spdlog::error("Failed to open: {}", result.error());
        } else {
            auto asset  = std::make_unique<PointsAsset>();
            asset->unit = std::move(points);

            auto& pointcloud = asset->unit;
            pointcloud->attach_renderer(renderer);
            pointcloud->set_overall_color(r, g, b);

            renderer.render_window();
            pointclouds[location] = std::move(asset);
        }
    }

    auto set_default_pointcloud_color(double r, double g, double b) noexcept {
        for (auto& [name, asset] : pointclouds) {
            if (asset->use_default) {
                asset->unit->set_overall_color(r, g, b);
            }
        }
        default_point_color = std::tie(r, g, b);
        renderer.render_window();
    }
};

auto AssetsManager::get_pointcloud_locations() const noexcept -> std::generator<std::string_view> {
    return pimpl->get_pointcloud_locations();
}
auto AssetsManager::get_pointcloud_handle(std::string const& key) noexcept
    -> std::optional<PointsHandle*> {
    if (pimpl->pointclouds.contains(key) && pimpl->pointclouds[key]->unit) {
        return pimpl->pointclouds[key]->unit.get();
    }
    return std::nullopt;
}

auto AssetsManager::update_renderer() const noexcept -> void { pimpl->renderer.render_window(); }

auto AssetsManager::open_pointcloud_file(std::string const& location) noexcept -> void {
    pimpl->open_pointcloud_file(location);
}
auto AssetsManager::clean_pointclouds() noexcept -> void {
    pimpl->clean_pointclouds(); //
}
auto AssetsManager::set_pointclouds_visibility(bool on) noexcept -> void {
    pimpl->set_pointclouds_visibility(on);
}
auto AssetsManager::set_default_point_color(double r, double g, double b) noexcept -> void {
    pimpl->set_default_pointcloud_color(r, g, b);
}

AssetsManager::AssetsManager(Renderer& renderer) noexcept
    : pimpl(std::make_unique<Impl>(renderer)) { }

AssetsManager::~AssetsManager() noexcept = default;
