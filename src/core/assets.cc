#include "assets.hh"
#include "core/renderer.hh"

#include <cassert>
#include <format>
#include <unordered_map>

using namespace pcs;
using namespace pcs::asset::internal;

struct AssetsManager::Impl {
    Renderer& renderer;

    std::unordered_map<std::string, std::unique_ptr<IAsset>> assets;
    std::vector<std::string> asset_order;
    std::size_t next_asset_id = 0;

    explicit Impl(Renderer& renderer) noexcept
        : renderer { renderer } { }

    auto create_asset_id() noexcept -> std::string {
        return std::format("asset-{}", next_asset_id++);
    }

    auto get_asset(std::string const& id) const noexcept -> IAsset const* {
        if (auto iter = assets.find(id); iter != assets.end()) {
            return iter->second.get();
        }
        return nullptr;
    }

    auto get_asset(std::string const& id) noexcept -> IAsset* {
        if (auto iter = assets.find(id); iter != assets.end()) {
            return iter->second.get();
        }
        return nullptr;
    }

    auto register_asset_internal(std::unique_ptr<IAsset> asset, std::string const& name,
        std::string const& location, bool persisted) noexcept -> std::string {
        asset->id        = create_asset_id();
        asset->name      = name;
        asset->location  = location;
        asset->persisted = persisted;

        asset->attach_unit(renderer);
        asset->set_visibility(asset->visible);

        auto id = asset->id;
        asset_order.emplace_back(id);
        assets[id] = std::move(asset);

        renderer.render_window();
        return id;
    }

    auto clean_assets() noexcept -> void {
        for (auto& [_, asset] : assets) {
            asset->release_unit(renderer);
        }

        assets.clear();
        asset_order.clear();
        renderer.render_window();
    }

    auto get_asset_ids() const noexcept -> std::generator<std::string_view> {
        for (auto const& id : asset_order) {
            if (assets.contains(id)) {
                co_yield std::string_view { id };
            }
        }
    }

    auto last_asset_id() const noexcept -> std::optional<std::string> {
        for (auto iter = asset_order.rbegin(); iter != asset_order.rend(); ++iter) {
            if (assets.contains(*iter)) {
                return *iter;
            }
        }
        return std::nullopt;
    }

    auto remove_asset(std::string const& id) noexcept -> bool {
        auto iter = assets.find(id);
        if (iter == assets.end()) {
            return false;
        }

        iter->second->release_unit(renderer);
        assets.erase(iter);
        std::erase(asset_order, id);

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

    auto clone_asset(std::string const& source_id, std::string const& target_name) noexcept
        -> std::expected<std::string, std::string> {
        auto* source = get_asset(source_id);
        if (source == nullptr) {
            return std::unexpected { "Source asset not found" };
        }

        auto cloned = source->clone(target_name);
        if (!cloned.has_value()) {
            return std::unexpected { cloned.error() };
        }

        auto id = register_asset_internal(
            std::move(*cloned), (*cloned)->name, (*cloned)->location, (*cloned)->persisted);
        return id;
    }

    auto save_asset(std::string const& id, std::string const& path) noexcept
        -> std::expected<void, std::string> {
        auto* asset = get_asset(id);
        if (asset == nullptr) {
            return std::unexpected { "Asset not found" };
        }

        return asset->save(path);
    }
};

auto AssetsManager::get_asset_internal(std::string const& id) noexcept -> IAsset* {
    return pimpl->get_asset(id);
}

auto AssetsManager::get_asset_internal(std::string const& id) const noexcept -> IAsset const* {
    return pimpl->get_asset(id);
}

auto AssetsManager::register_asset_internal(std::unique_ptr<IAsset> asset, std::string const& name,
    std::string const& location, bool persisted) noexcept -> std::string {
    return pimpl->register_asset_internal(std::move(asset), name, location, persisted);
}

auto AssetsManager::get_asset_ids() const noexcept -> std::generator<std::string_view> {
    return pimpl->get_asset_ids();
}

auto AssetsManager::last_asset_id() const noexcept -> std::string {
    return pimpl->last_asset_id().value_or(std::string { });
}

auto AssetsManager::get_asset_name(std::string const& id) const noexcept -> std::string {
    auto* asset = pimpl->get_asset(id);
    assert(asset != nullptr && "asset id must exist");
    return asset != nullptr ? asset->name : std::string { };
}

auto AssetsManager::get_asset_type(std::string const& id) const noexcept -> std::type_index {
    auto* asset = pimpl->get_asset(id);
    assert(asset != nullptr && "asset id must exist");
    return asset != nullptr ? asset->type_index() : std::type_index { typeid(void) };
}

auto AssetsManager::get_asset_kind(std::string const& id) const noexcept -> std::string_view {
    auto* asset = pimpl->get_asset(id);
    assert(asset != nullptr && "asset id must exist");
    return asset != nullptr ? asset->kind() : std::string_view { };
}

auto AssetsManager::get_asset_display_name(std::string const& id) const noexcept -> std::string {
    auto* asset = pimpl->get_asset(id);
    auto kind   = get_asset_kind(id);
    assert(asset != nullptr && "asset id must exist");
    if (asset == nullptr) {
        return { };
    }

    auto display = std::format("{} [{}]", asset->name, kind);
    if (!asset->persisted) {
        display += " [memory]";
    }
    return display;
}

auto AssetsManager::get_asset_path(std::string const& id) const noexcept -> std::string {
    auto* asset = pimpl->get_asset(id);
    assert(asset != nullptr && "asset id must exist");
    if (asset == nullptr) {
        return { };
    }
    return asset->location.empty() ? "<memory>" : asset->location;
}

auto AssetsManager::is_asset_visible(std::string const& id) const noexcept -> bool {
    auto* asset = pimpl->get_asset(id);
    assert(asset != nullptr && "asset id must exist");
    return asset != nullptr && asset->visible;
}

auto AssetsManager::set_asset_visibility(std::string const& id, bool on) noexcept -> bool {
    return pimpl->set_asset_visibility(id, on);
}

auto AssetsManager::clone_asset(std::string const& source_id,
    std::string const& target_name) noexcept -> std::expected<std::string, std::string> {
    return pimpl->clone_asset(source_id, target_name);
}

auto AssetsManager::save_asset(std::string const& id, std::string const& path) noexcept
    -> std::expected<void, std::string> {
    return pimpl->save_asset(id, path);
}

auto AssetsManager::remove_asset(std::string const& id) noexcept -> bool {
    return pimpl->remove_asset(id);
}

auto AssetsManager::update_renderer() const noexcept -> void { pimpl->renderer.render_window(); }

auto AssetsManager::clean_assets() noexcept -> void { pimpl->clean_assets(); }

AssetsManager::AssetsManager(Renderer& renderer) noexcept
    : pimpl(std::make_unique<Impl>(renderer)) { }

AssetsManager::~AssetsManager() noexcept = default;
