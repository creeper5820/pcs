#include "points.hh"

#include <format>

using namespace pcs::event;

auto MakePointsUnit::exec() noexcept -> Result {

    auto handle = std::make_unique<PointsHandle>();

    auto result = handle->load_from_filesystem(path);
    if (!result.has_value()) {
        const auto error = result.error();
        return std::unexpected {
            std::format("Points unit created failed: {}", error),
        };
    }

    return handle;
}

auto MakePointsUnit::redo() noexcept -> Result { return exec(); }
