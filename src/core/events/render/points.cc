#include "points.hh"

using namespace pcs::event;

auto MakePointsUnit::runtime_exec(std::unique_ptr<Context> context) noexcept -> Result {

    const auto& path = context->path;
    const auto& name = context->name;

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
