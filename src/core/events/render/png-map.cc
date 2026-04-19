#include "png-map.hh"

#include <format>

using namespace pcs::event;

auto MakePngMapUnit::runtime_exec(std::unique_ptr<Context> context) noexcept -> Result {
    const auto& path = context->path;

    auto handle = std::make_unique<pcs::PngMapHandle>();
    auto result = handle->load_from_filesystem(path);

    if (!result.has_value()) {
        return std::unexpected {
            std::format("PNG map unit created failed: {}", result.error()),
        };
    }

    return handle;
}
