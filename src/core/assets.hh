#pragma once

#include "core/renderer.hh"
#include "utility/pimpl.hh"

#include <concepts>
#include <expected>
#include <filesystem>
#include <generator>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <typeindex>
#include <utility>

namespace pcs {

namespace asset::internal {
    template <class Handle>
    concept asset_trait = requires {
        {
            std::declval<Handle&>().attach_renderer(std::declval<Renderer&>())
        } noexcept -> std::same_as<void>;
        {
            std::declval<Handle&>().detach_renderer(std::declval<Renderer&>())
        } noexcept -> std::same_as<void>;
        {
            std::declval<Handle&>().set_visibility(std::declval<bool>())
        } noexcept -> std::same_as<void>;
        {
            std::declval<Handle const&>().clone(std::declval<std::string const&>())
        } noexcept -> std::convertible_to<std::expected<std::unique_ptr<Handle>, std::string>>;
        {
            std::declval<Handle&>().save_into_filesystem(std::declval<std::string const&>())
        } -> std::convertible_to<std::expected<void, std::string_view>>;
    };

}

class AssetsManager final {
    PCS_PIMPL_DEFINITION(AssetsManager);

public:
    explicit AssetsManager(Renderer&) noexcept;

    auto update_renderer() const noexcept -> void;

    auto clean_assets() noexcept -> void;

    auto get_asset_ids() const noexcept -> std::generator<std::string_view>;
    auto last_asset_id() const noexcept -> std::string;
    auto get_asset_kind(std::string const& id) const noexcept -> std::string_view;
    auto get_asset_type(std::string const& id) const noexcept -> std::type_index;
    auto get_asset_display_name(std::string const& id) const noexcept -> std::string;
    auto get_asset_name(std::string const& id) const noexcept -> std::string;
    auto get_asset_path(std::string const& id) const noexcept -> std::string;
    auto is_asset_visible(std::string const& id) const noexcept -> bool;

    template <class Handle>
    auto get_handle(std::string const& id) noexcept -> std::optional<Handle*> {
        auto* asset = get_asset_internal(id);
        if (asset == nullptr || asset->type_index() != std::type_index { typeid(Handle) }) {
            return std::nullopt;
        }
        return static_cast<Handle*>(asset->get_ptr());
    }

    template <class Handle>
    auto register_asset(std::unique_ptr<Handle> handle, std::string const& name,
        std::string const& location, bool persisted = true) noexcept -> std::string {
        auto asset_holder = std::make_unique<AssetHolder<Handle>>(std::move(handle));
        return register_asset_internal(std::move(asset_holder), name, location, persisted);
    }

    auto clone_asset(std::string const& source_id, std::string const& target_name = "") noexcept
        -> std::expected<std::string, std::string>;

    auto save_asset(std::string const& id, std::string const& path) noexcept
        -> std::expected<void, std::string>;

    auto set_asset_visibility(std::string const& id, bool on) noexcept -> bool;
    auto remove_asset(std::string const& id) noexcept -> bool;

private:
    struct IAsset {
        std::string id;
        std::string name;
        std::string location;
        bool visible   = true;
        bool persisted = true;

        virtual ~IAsset() = default;

        virtual auto release_unit(Renderer&) noexcept -> void       = 0;
        virtual auto attach_unit(Renderer&) noexcept -> void        = 0;
        virtual auto set_visibility(bool on) noexcept -> void       = 0;
        virtual auto kind() const noexcept -> std::string_view      = 0;
        virtual auto type_index() const noexcept -> std::type_index = 0;
        virtual auto get_ptr() noexcept -> void*                    = 0;
        virtual auto get_ptr() const noexcept -> void const*        = 0;
        virtual auto clone(std::string const& target_name) const noexcept
            -> std::expected<std::unique_ptr<IAsset>, std::string>                              = 0;
        virtual auto save(std::string const& path) noexcept -> std::expected<void, std::string> = 0;
    };

    template <class Handle>
    struct AssetHolder final : IAsset {
        static_assert(asset::internal::asset_trait<Handle>,
            "Handle does not satisfy asset_trait requirements");

        std::unique_ptr<Handle> handle;

        explicit AssetHolder(std::unique_ptr<Handle> h) noexcept
            : handle { std::move(h) } { }

        auto type_index() const noexcept -> std::type_index override {
            return std::type_index { typeid(Handle) };
        }

        auto kind() const noexcept -> std::string_view override { return Handle::kKind; }

        auto get_ptr() noexcept -> void* override { return handle.get(); }

        auto get_ptr() const noexcept -> void const* override { return handle.get(); }

        auto release_unit(Renderer& renderer) noexcept -> void override {
            handle->detach_renderer(renderer);
        }

        auto attach_unit(Renderer& renderer) noexcept -> void override {
            handle->attach_renderer(renderer);
        }

        auto set_visibility(bool on) noexcept -> void override {
            visible = on;
            handle->set_visibility(on);
        }

        auto clone(std::string const& target_name) const noexcept
            -> std::expected<std::unique_ptr<IAsset>, std::string> override {
            auto result = handle->clone(target_name.empty() ? name : target_name);
            if (!result.has_value()) {
                return std::unexpected { result.error() };
            }

            auto cloned       = std::make_unique<AssetHolder<Handle>>(std::move(result.value()));
            cloned->name      = target_name.empty() ? (name + "-copy") : target_name;
            cloned->location  = { };
            cloned->persisted = false;
            return cloned;
        }

        auto save(std::string const& path) noexcept -> std::expected<void, std::string> override {
            auto result = handle->save_into_filesystem(path);
            if (!result.has_value()) {
                return std::unexpected { std::string(result.error()) };
            }

            location  = path;
            name      = std::filesystem::path(path).filename().string();
            persisted = true;
            return { };
        }
    };
    auto get_asset_internal(std::string const& id) noexcept -> IAsset*;
    auto get_asset_internal(std::string const& id) const noexcept -> IAsset const*;
    auto register_asset_internal(std::unique_ptr<IAsset>, std::string const& name,
        std::string const& location, bool persisted) noexcept -> std::string;
};

}
