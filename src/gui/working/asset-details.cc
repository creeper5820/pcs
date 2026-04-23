#include "gui/working/asset-details.hh"

namespace pcs::gui::working {

auto AssetDetailsRegistry::register_provider(std::type_index type, Provider provider) noexcept
    -> void {
    providers[type] = std::move(provider);
}

auto AssetDetailsRegistry::provide(std::type_index type, pcs::AssetsManager& assets,
    std::string const& id) const noexcept -> std::optional<AssetDetails> {
    auto iter = providers.find(type);
    if (iter == providers.end() || !iter->second) {
        return std::nullopt;
    }

    return iter->second(assets, id);
}

}
