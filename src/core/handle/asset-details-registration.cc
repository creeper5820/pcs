#include "core/handle/asset-details-registration.hh"

#include "gui/working/asset-details.hh"

namespace pcs::handle::points {
auto register_details_provider(gui::working::AssetDetailsRegistry&) noexcept -> void;
}

namespace pcs::handle::model {
auto register_details_provider(gui::working::AssetDetailsRegistry&) noexcept -> void;
}

namespace pcs::handle::png_map {
auto register_details_provider(gui::working::AssetDetailsRegistry&) noexcept -> void;
}

namespace pcs::handle {

auto register_all_asset_details_providers(gui::working::AssetDetailsRegistry& registry) noexcept
    -> void {
    points::register_details_provider(registry);
    model::register_details_provider(registry);
    png_map::register_details_provider(registry);
}

}
