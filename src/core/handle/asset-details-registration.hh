#pragma once

namespace pcs::gui::working {
class AssetDetailsRegistry;
}

namespace pcs::handle {

auto register_all_asset_details_providers(gui::working::AssetDetailsRegistry&) noexcept -> void;

}
