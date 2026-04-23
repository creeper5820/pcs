#pragma once

#include <functional>
#include <optional>
#include <string>
#include <typeindex>
#include <unordered_map>

#include <QString>

namespace pcs {
class AssetsManager;
}

namespace pcs::gui::working {

struct AssetDetails {
    QString type;
    QString size;
    QString info;
};

class AssetDetailsRegistry {
public:
    using Provider =
        std::function<std::optional<AssetDetails>(pcs::AssetsManager&, std::string const&)>;

    auto register_provider(std::type_index, Provider) noexcept -> void;
    auto provide(std::type_index, pcs::AssetsManager&, std::string const&) const noexcept
        -> std::optional<AssetDetails>;

private:
    std::unordered_map<std::type_index, Provider> providers;
};

}
