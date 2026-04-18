#include "assets.hh"

#include "core/events/process/model-to-pointcloud.hh"
#include "core/events/process/pointcloud-to-png-map.hh"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <format>
#include <ranges>
#include <spdlog/spdlog.h>
#include <tuple>
#include <unordered_map>
#include <vector>

using namespace pcs;

namespace {

auto kind_label(AssetKind kind) noexcept -> std::string_view {
    switch (kind) {
    case AssetKind::Pointcloud:
        return "PointCloud";
    case AssetKind::Model:
        return "Model";
    case AssetKind::PngMap:
        return "PNG Map";
    }

    return "Unknown";
}

auto normalized_extension(std::string const& location) -> std::string {
    auto extension = std::filesystem::path(location).extension().string();
    std::ranges::transform(extension, extension.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return extension;
}

auto inferred_name(std::string const& location, std::string_view fallback) -> std::string {
    if (location.empty()) {
        return std::string { fallback };
    }

    return std::filesystem::path(location).filename().string();
}

}

struct AssetsManager::Impl {
    Renderer& renderer;

    explicit Impl(Renderer& renderer) noexcept
        : renderer { renderer } { }

    struct Asset {
        std::string id;
        std::string name;
        std::string location;
        bool visible   = true;
        bool persisted = true;

        virtual ~Asset() = default;

        virtual auto kind() const noexcept -> AssetKind       = 0;
        virtual auto release_unit(Renderer&) noexcept -> void = 0;
        virtual auto set_visibility(bool on) noexcept -> void = 0;
    };

    struct PointcloudAsset final : Asset {
        std::unique_ptr<PointsHandle> unit;

        auto kind() const noexcept -> AssetKind override { return AssetKind::Pointcloud; }

        auto release_unit(Renderer& renderer) noexcept -> void override {
            if (unit) {
                unit->detach_renderer(renderer);
            }
        }

        auto set_visibility(bool on) noexcept -> void override {
            visible = on;
            unit->set_visibility(on);
        }
    };

    struct ModelAsset final : Asset {
        std::unique_ptr<ModelHandle> unit;

        auto kind() const noexcept -> AssetKind override { return AssetKind::Model; }

        auto release_unit(Renderer& renderer) noexcept -> void override {
            if (unit) {
                unit->detach_renderer(renderer);
            }
        }

        auto set_visibility(bool on) noexcept -> void override {
            visible = on;
            unit->set_visibility(on);
        }
    };

    struct PngMapAsset final : Asset {
        std::unique_ptr<PngMapHandle> unit;

        auto kind() const noexcept -> AssetKind override { return AssetKind::PngMap; }

        auto release_unit(Renderer& renderer) noexcept -> void override {
            if (unit) {
                unit->detach_renderer(renderer);
            }
        }

        auto set_visibility(bool on) noexcept -> void override {
            visible = on;
            unit->set_visibility(on);
        }
    };

    std::tuple<double, double, double> default_point_color { 1.0, 1.0, 1.0 };

    std::unordered_map<std::string, std::unique_ptr<Asset>> assets;
    std::vector<std::string> asset_order;
    std::size_t next_asset_id = 0;
    std::unordered_map<std::string, std::string> generated_pointcloud_asset_by_source;
    std::unordered_map<std::string, std::string> generated_png_asset_by_source;

    auto create_asset_id(AssetKind kind) noexcept -> std::string {
        const auto prefix = [&] {
            switch (kind) {
            case AssetKind::Pointcloud:
                return "pointcloud";
            case AssetKind::Model:
                return "model";
            case AssetKind::PngMap:
                return "png-map";
            }
            return "asset";
        }();

        return std::format("{}-{}", prefix, next_asset_id++);
    }

    auto get_asset(std::string const& id) const noexcept -> Asset const* {
        if (auto iter = assets.find(id); iter != assets.end()) {
            return iter->second.get();
        }

        return nullptr;
    }

    auto get_asset(std::string const& id) noexcept -> Asset* {
        if (auto iter = assets.find(id); iter != assets.end()) {
            return iter->second.get();
        }

        return nullptr;
    }

    auto get_pointcloud_asset(std::string const& id) noexcept -> PointcloudAsset* {
        auto* asset = get_asset(id);
        if (asset == nullptr || asset->kind() != AssetKind::Pointcloud) {
            return nullptr;
        }

        return static_cast<PointcloudAsset*>(asset);
    }

    auto get_model_asset(std::string const& id) noexcept -> ModelAsset* {
        auto* asset = get_asset(id);
        if (asset == nullptr || asset->kind() != AssetKind::Model) {
            return nullptr;
        }

        return static_cast<ModelAsset*>(asset);
    }

    auto get_png_map_asset(std::string const& id) noexcept -> PngMapAsset* {
        auto* asset = get_asset(id);
        if (asset == nullptr || asset->kind() != AssetKind::PngMap) {
            return nullptr;
        }

        return static_cast<PngMapAsset*>(asset);
    }

    auto get_asset_path_label(Asset const& asset) const -> std::string {
        if (asset.location.empty()) {
            return "<memory>";
        }

        return asset.location;
    }

    auto get_asset_display_name(Asset const& asset) const -> std::string {
        auto label = std::format("{} [{}]", asset.name, kind_label(asset.kind()));
        if (!asset.persisted) {
            label += " [memory]";
        }
        return label;
    }

    auto register_pointcloud(std::unique_ptr<PointsHandle> pointcloud, std::string const& name,
        std::string const& location, bool persisted) noexcept -> std::string {
        auto asset       = std::make_unique<PointcloudAsset>();
        asset->id        = create_asset_id(AssetKind::Pointcloud);
        asset->name      = name;
        asset->location  = location;
        asset->persisted = persisted;
        asset->unit      = std::move(pointcloud);

        auto [r, g, b] = default_point_color;
        asset->unit->attach_renderer(renderer);
        asset->unit->set_overall_color(r, g, b);
        asset->unit->set_visibility(asset->visible);

        auto id = asset->id;
        asset_order.emplace_back(id);
        assets[id] = std::move(asset);

        renderer.render_window();
        return id;
    }

    auto register_model(std::unique_ptr<ModelHandle> model, std::string const& name,
        std::string const& location) noexcept -> std::string {
        auto asset      = std::make_unique<ModelAsset>();
        asset->id       = create_asset_id(AssetKind::Model);
        asset->name     = name;
        asset->location = location;
        asset->unit     = std::move(model);

        asset->unit->attach_renderer(renderer);
        asset->unit->set_visibility(asset->visible);

        auto id = asset->id;
        asset_order.emplace_back(id);
        assets[id] = std::move(asset);

        renderer.render_window();
        return id;
    }

    auto register_png_map(std::unique_ptr<PngMapHandle> png_map, std::string const& name,
        std::string const& location, bool persisted) noexcept -> std::string {
        auto asset       = std::make_unique<PngMapAsset>();
        asset->id        = create_asset_id(AssetKind::PngMap);
        asset->name      = name;
        asset->location  = location;
        asset->persisted = persisted;
        asset->unit      = std::move(png_map);

        asset->unit->attach_renderer(renderer);
        asset->unit->set_visibility(asset->visible);

        auto id = asset->id;
        asset_order.emplace_back(id);
        assets[id] = std::move(asset);

        renderer.render_window();
        return id;
    }

    auto open_pointcloud_file(std::string const& location) noexcept -> void {
        auto pointcloud = std::make_unique<PointsHandle>();
        auto result     = pointcloud->load_from_filesystem(location);

        if (!result.has_value()) {
            spdlog::error("Failed to open pointcloud: {}", result.error());
            return;
        }

        register_pointcloud(
            std::move(pointcloud), inferred_name(location, "pointcloud.pcd"), location, true);
    }

    auto open_model_file(std::string const& location) noexcept -> void {
        auto model  = std::make_unique<ModelHandle>();
        auto result = model->load_from_filesystem(location);

        if (!result.has_value()) {
            spdlog::error("Failed to open model: {}", result.error());
            return;
        }

        register_model(std::move(model), inferred_name(location, "model.obj"), location);
    }

    auto open_file(std::string const& location) noexcept -> void {
        const auto extension = normalized_extension(location);

        if (extension == ".pcd") {
            open_pointcloud_file(location);
            return;
        }

        if (extension == ".obj") {
            open_model_file(location);
            return;
        }

        spdlog::error("Unsupported asset file: {}", location);
    }

    auto clean_assets() noexcept -> void {
        for (auto& [_, asset] : assets) {
            asset->release_unit(renderer);
        }

        assets.clear();
        asset_order.clear();
        generated_pointcloud_asset_by_source.clear();
        generated_png_asset_by_source.clear();
        renderer.render_window();
    }

    auto get_asset_ids() const noexcept -> std::generator<std::string_view> {
        for (auto const& id : asset_order) {
            if (assets.contains(id)) {
                co_yield std::string_view { id };
            }
        }
    }

    auto create_pointcloud_asset_from_positions(std::string const& source_id,
        std::vector<event::ConvertModelToPointcloud::Position> const& points) noexcept
        -> std::expected<std::string, std::string> {
        auto* source_asset = get_asset(source_id);
        if (source_asset == nullptr) {
            return std::unexpected { "Source asset is not loaded" };
        }

        auto pointcloud  = std::make_unique<PointsHandle>();
        auto load_result = pointcloud->load_from_positions(points);
        if (!load_result.has_value()) {
            return std::unexpected { std::string { load_result.error() } };
        }

        auto derived_name = std::filesystem::path(source_asset->name);
        if (derived_name.empty()) {
            derived_name = "converted-pointcloud.pcd";
        } else {
            derived_name.replace_extension(".pcd");
        }

        auto new_id = register_pointcloud(
            std::move(pointcloud), derived_name.filename().string(), { }, false);

        return new_id;
    }

    auto replace_pointcloud_asset_data(std::string const& id,
        std::vector<event::ConvertModelToPointcloud::Position> const& points) noexcept
        -> std::expected<void, std::string> {
        auto* asset = get_pointcloud_asset(id);
        if (asset == nullptr) {
            return std::unexpected { "Pointcloud asset is not loaded" };
        }

        auto replacement = std::make_unique<PointsHandle>();
        auto load_result = replacement->load_from_positions(points);
        if (!load_result.has_value()) {
            return std::unexpected { std::string { load_result.error() } };
        }

        const auto [r, g, b, a] = asset->unit->get_overall_color();
        const auto visible      = asset->visible;

        asset->release_unit(renderer);
        asset->unit = std::move(replacement);

        asset->unit->attach_renderer(renderer);
        asset->unit->set_overall_color(r, g, b, a);
        asset->unit->set_visibility(visible);
        renderer.render_window();
        return { };
    }

    auto upsert_generated_pointcloud(std::string const& source_id,
        std::vector<event::ConvertModelToPointcloud::Position> const& points) noexcept
        -> std::expected<std::string, std::string> {
        if (get_asset(source_id) == nullptr) {
            return std::unexpected { "Source asset is not loaded" };
        }

        if (auto iter = generated_pointcloud_asset_by_source.find(source_id);
            iter != generated_pointcloud_asset_by_source.end()) {
            const auto& existing_id = iter->second;
            if (get_pointcloud_asset(existing_id) != nullptr) {
                auto replace_result = replace_pointcloud_asset_data(existing_id, points);
                if (!replace_result.has_value()) {
                    return std::unexpected { replace_result.error() };
                }
                return existing_id;
            }

            generated_pointcloud_asset_by_source.erase(iter);
        }

        auto created = create_pointcloud_asset_from_positions(source_id, points);
        if (!created.has_value()) {
            return std::unexpected { created.error() };
        }

        generated_pointcloud_asset_by_source[source_id] = *created;
        return created;
    }

    auto convert_model_to_pointcloud(
        std::string const& id, pcs::ModelToPointcloudParameters const& parameters) noexcept
        -> std::expected<std::string, std::string> {
        auto* asset = get_model_asset(id);
        if (asset == nullptr || asset->unit == nullptr) {
            return std::unexpected { "Model asset is not loaded" };
        }

        auto context        = std::make_unique<event::ConvertModelToPointcloud::Context>();
        context->poly_data  = asset->unit->poly_data();
        context->parameters = parameters;

        auto result = event::ConvertModelToPointcloud::runtime_exec(std::move(context));
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        return upsert_generated_pointcloud(id, result.value());
    }

    auto create_png_map_asset_from_data(std::string const& source_id,
        PngMapData const& data) noexcept -> std::expected<std::string, std::string> {
        auto* source_asset = get_asset(source_id);
        if (source_asset == nullptr) {
            return std::unexpected { "Source asset is not loaded" };
        }

        auto png_map     = std::make_unique<PngMapHandle>();
        auto load_result = png_map->load_from_data(data);
        if (!load_result.has_value()) {
            return std::unexpected { std::string { load_result.error() } };
        }

        auto derived_name = std::filesystem::path(source_asset->name);
        if (derived_name.empty()) {
            derived_name = "generated-map.png";
        } else {
            derived_name.replace_extension(".png");
        }

        auto new_id =
            register_png_map(std::move(png_map), derived_name.filename().string(), { }, false);

        return new_id;
    }

    auto replace_png_map_asset_data(std::string const& id, PngMapData const& data) noexcept
        -> std::expected<void, std::string> {
        auto* asset = get_png_map_asset(id);
        if (asset == nullptr) {
            return std::unexpected { "PNG map asset is not loaded" };
        }

        auto replacement = std::make_unique<PngMapHandle>();
        auto load_result = replacement->load_from_data(data);
        if (!load_result.has_value()) {
            return std::unexpected { std::string { load_result.error() } };
        }

        const auto visible = asset->visible;
        asset->release_unit(renderer);
        asset->unit = std::move(replacement);

        asset->unit->attach_renderer(renderer);
        asset->unit->set_visibility(visible);
        renderer.render_window();
        return { };
    }

    auto upsert_generated_png_map(std::string const& source_id, PngMapData const& data) noexcept
        -> std::expected<std::string, std::string> {
        if (get_asset(source_id) == nullptr) {
            return std::unexpected { "Source asset is not loaded" };
        }

        if (auto iter = generated_png_asset_by_source.find(source_id);
            iter != generated_png_asset_by_source.end()) {
            const auto& existing_id = iter->second;
            if (get_png_map_asset(existing_id) != nullptr) {
                auto replace_result = replace_png_map_asset_data(existing_id, data);
                if (!replace_result.has_value()) {
                    return std::unexpected { replace_result.error() };
                }
                return existing_id;
            }

            generated_png_asset_by_source.erase(iter);
        }

        auto created = create_png_map_asset_from_data(source_id, data);
        if (!created.has_value()) {
            return std::unexpected { created.error() };
        }

        generated_png_asset_by_source[source_id] = *created;
        return created;
    }

    auto generate_png_map_from_pointcloud(std::string const& id,
        PngMapParameters const& parameters) noexcept -> std::expected<std::string, std::string> {
        auto* asset = get_pointcloud_asset(id);
        if (asset == nullptr || asset->unit == nullptr) {
            return std::unexpected { "Pointcloud asset is not loaded" };
        }

        auto context        = std::make_unique<event::ConvertPointcloudToPngMap::Context>();
        context->points     = asset->unit->get_positions();
        context->parameters = parameters;

        auto result = event::ConvertPointcloudToPngMap::runtime_exec(std::move(context));
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        return upsert_generated_png_map(id, result.value());
    }

    auto save_pointcloud_asset(std::string const& id, std::string const& path) noexcept
        -> std::expected<void, std::string> {
        auto* asset = get_pointcloud_asset(id);
        if (asset == nullptr || asset->unit == nullptr) {
            return std::unexpected { "Pointcloud asset is not loaded" };
        }

        auto result = asset->unit->save_into_filesystem(path);
        if (!result.has_value()) {
            return std::unexpected { std::string { result.error() } };
        }

        asset->location  = path;
        asset->name      = std::filesystem::path(path).filename().string();
        asset->persisted = true;

        return { };
    }

    auto save_png_map_asset(std::string const& id, std::string const& path) noexcept
        -> std::expected<void, std::string> {
        auto* asset = get_png_map_asset(id);
        if (asset == nullptr || asset->unit == nullptr) {
            return std::unexpected { "PNG map asset is not loaded" };
        }

        auto result = asset->unit->save_into_filesystem(path);
        if (!result.has_value()) {
            return std::unexpected { std::string { result.error() } };
        }

        asset->location  = path;
        asset->name      = std::filesystem::path(path).filename().string();
        asset->persisted = true;
        return { };
    }

    auto remove_asset(std::string const& id) noexcept -> bool {
        auto iter = assets.find(id);
        if (iter == assets.end()) {
            return false;
        }

        iter->second->release_unit(renderer);
        assets.erase(iter);
        std::erase(asset_order, id);

        const auto prune_generated_mapping = [id](auto& generated_assets_by_source) {
            if (auto source_iter = generated_assets_by_source.find(id);
                source_iter != generated_assets_by_source.end()) {
                generated_assets_by_source.erase(source_iter);
            }

            for (auto mapping_iter = generated_assets_by_source.begin();
                mapping_iter != generated_assets_by_source.end();) {
                if (mapping_iter->second == id) {
                    mapping_iter = generated_assets_by_source.erase(mapping_iter);
                    continue;
                }

                ++mapping_iter;
            }
        };

        prune_generated_mapping(generated_pointcloud_asset_by_source);
        prune_generated_mapping(generated_png_asset_by_source);

        renderer.render_window();
        return true;
    }

    auto set_asset_visibility(std::string const& id, bool on) noexcept -> bool {
        auto* asset = get_asset(id);
        if (asset == nullptr) {
            return false;
        }

        asset->set_visibility(on);
        renderer.render_window();
        return true;
    }

    auto set_default_pointcloud_color(double r, double g, double b) noexcept {
        default_point_color = std::tie(r, g, b);

        for (auto const& id : asset_order) {
            auto* asset = get_pointcloud_asset(id);
            if (asset != nullptr && asset->unit != nullptr) {
                const auto alpha = std::get<3>(asset->unit->get_overall_color());
                asset->unit->set_overall_color(r, g, b, alpha);
            }
        }

        renderer.render_window();
    }
};

auto AssetsManager::get_asset_ids() const noexcept -> std::generator<std::string_view> {
    return pimpl->get_asset_ids();
}

auto AssetsManager::get_asset_kind(std::string const& id) const noexcept
    -> std::optional<AssetKind> {
    if (auto* asset = pimpl->get_asset(id)) {
        return asset->kind();
    }

    return std::nullopt;
}

auto AssetsManager::get_asset_display_name(std::string const& id) const noexcept
    -> std::optional<std::string> {
    if (auto* asset = pimpl->get_asset(id)) {
        return pimpl->get_asset_display_name(*asset);
    }

    return std::nullopt;
}

auto AssetsManager::get_asset_name(std::string const& id) const noexcept
    -> std::optional<std::string> {
    if (auto* asset = pimpl->get_asset(id)) {
        return asset->name;
    }

    return std::nullopt;
}

auto AssetsManager::get_asset_path(std::string const& id) const noexcept
    -> std::optional<std::string> {
    if (auto* asset = pimpl->get_asset(id)) {
        return pimpl->get_asset_path_label(*asset);
    }

    return std::nullopt;
}

auto AssetsManager::is_asset_visible(std::string const& id) const noexcept -> std::optional<bool> {
    if (auto* asset = pimpl->get_asset(id)) {
        return asset->visible;
    }

    return std::nullopt;
}

auto AssetsManager::get_pointcloud_handle(std::string const& id) noexcept
    -> std::optional<PointsHandle*> {
    if (auto* asset = pimpl->get_pointcloud_asset(id)) {
        return asset->unit.get();
    }

    return std::nullopt;
}

auto AssetsManager::get_model_handle(std::string const& id) noexcept
    -> std::optional<ModelHandle*> {
    if (auto* asset = pimpl->get_model_asset(id)) {
        return asset->unit.get();
    }

    return std::nullopt;
}

auto AssetsManager::get_png_map_handle(std::string const& id) noexcept
    -> std::optional<PngMapHandle*> {
    if (auto* asset = pimpl->get_png_map_asset(id)) {
        return asset->unit.get();
    }

    return std::nullopt;
}

auto AssetsManager::set_asset_visibility(std::string const& id, bool on) noexcept -> bool {
    return pimpl->set_asset_visibility(id, on);
}

auto AssetsManager::convert_model_to_pointcloud(
    std::string const& id, ModelToPointcloudParameters const& parameters) noexcept
    -> std::expected<std::string, std::string> {
    return pimpl->convert_model_to_pointcloud(id, parameters);
}

auto AssetsManager::generate_png_map_from_pointcloud(std::string const& id,
    PngMapParameters const& parameters) noexcept -> std::expected<std::string, std::string> {
    return pimpl->generate_png_map_from_pointcloud(id, parameters);
}

auto AssetsManager::create_png_map_asset_from_data(std::string const& source_id,
    PngMapData const& data) noexcept -> std::expected<std::string, std::string> {
    return pimpl->create_png_map_asset_from_data(source_id, data);
}

auto AssetsManager::upsert_generated_png_map(std::string const& source_id,
    PngMapData const& data) noexcept -> std::expected<std::string, std::string> {
    return pimpl->upsert_generated_png_map(source_id, data);
}

auto AssetsManager::replace_png_map_asset_data(
    std::string const& id, PngMapData const& data) noexcept -> std::expected<void, std::string> {
    return pimpl->replace_png_map_asset_data(id, data);
}

auto AssetsManager::save_pointcloud_asset(std::string const& id, std::string const& path) noexcept
    -> std::expected<void, std::string> {
    return pimpl->save_pointcloud_asset(id, path);
}

auto AssetsManager::save_png_map_asset(std::string const& id, std::string const& path) noexcept
    -> std::expected<void, std::string> {
    return pimpl->save_png_map_asset(id, path);
}

auto AssetsManager::remove_asset(std::string const& id) noexcept -> bool {
    return pimpl->remove_asset(id);
}

auto AssetsManager::update_renderer() const noexcept -> void { pimpl->renderer.render_window(); }

auto AssetsManager::open_file(std::string const& location) noexcept -> void {
    pimpl->open_file(location);
}

auto AssetsManager::clean_assets() noexcept -> void { pimpl->clean_assets(); }

auto AssetsManager::set_default_point_color(double r, double g, double b) noexcept -> void {
    pimpl->set_default_pointcloud_color(r, g, b);
}

AssetsManager::AssetsManager(Renderer& renderer) noexcept
    : pimpl(std::make_unique<Impl>(renderer)) { }

AssetsManager::~AssetsManager() noexcept = default;
