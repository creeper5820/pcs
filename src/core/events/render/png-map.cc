#include "png-map.hh"

#include <format>

using namespace pcs::event;

auto MakePngMapUnit::exec() noexcept -> Result {
    auto handle = std::make_unique<pcs::PngMapHandle>();
    auto result = handle->load_from_filesystem(path);

    if (!result.has_value()) {
        return std::unexpected {
            std::format("PNG map unit created failed: {}", result.error()),
        };
    }

    return handle;
}

auto MakePngMapUnit::redo() noexcept -> Result { return exec(); }
