#include "gui/working/asset-details.hh"

namespace pcs::gui::working {

auto AssetDetailsRegistry::register_provider(pcs::AssetKind kind, Provider provider) noexcept
    -> void {
    providers[kind] = std::move(provider);
}

auto AssetDetailsRegistry::provide(pcs::AssetKind kind, pcs::AssetsManager& assets,
    std::string const& id) const noexcept -> std::optional<AssetDetails> {
    auto iter = providers.find(kind);
    if (iter == providers.end() || !iter->second) {
        return std::nullopt;
    }

    return iter->second(assets, id);
}

}
