#pragma once

#include "core/assets.hh"

#include <optional>
#include <string>
#include <functional>
#include <unordered_map>

#include <QString>

namespace pcs::gui::working {

struct AssetDetails {
    QString type;
    QString size;
    QString info;
};

class AssetDetailsRegistry {
public:
    using Provider = std::function<std::optional<AssetDetails>(pcs::AssetsManager&, std::string const&)>;

    auto register_provider(pcs::AssetKind, Provider) noexcept -> void;
    auto provide(pcs::AssetKind, pcs::AssetsManager&, std::string const&) const noexcept
        -> std::optional<AssetDetails>;

private:
    std::unordered_map<pcs::AssetKind, Provider> providers;
};

}
