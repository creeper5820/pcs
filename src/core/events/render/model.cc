#include "model.hh"

#include <format>

using namespace pcs::event;

auto MakeModelUnit::exec() noexcept -> Result {
    auto handle = std::make_unique<pcs::ModelHandle>();
    auto result = handle->load_from_filesystem(path);

    if (!result.has_value()) {
        return std::unexpected {
            std::format("Model unit created failed: {}", result.error()),
        };
    }

    return handle;
}

auto MakeModelUnit::redo() noexcept -> Result { return exec(); }
