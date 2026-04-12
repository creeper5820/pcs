#include "core/handle/model.hh"

#include <filesystem>
#include <fstream>
#include <iostream>

auto main() -> int {
    auto path = std::filesystem::temp_directory_path() / "pcs-model-without-mtl.obj";

    {
        auto output = std::ofstream { path };
        output << "mtllib missing-material.mtl\n";
        output << "o triangle\n";
        output << "v 0 0 0\n";
        output << "v 1 0 0\n";
        output << "v 0 1 0\n";
        output << "f 1 2 3\n";
    }

    auto cleanup = [&] { std::filesystem::remove(path); };

    auto handle = pcs::ModelHandle { };
    auto result = handle.load_from_filesystem(path.string());
    if (!result.has_value()) {
        cleanup();
        std::cerr << "load failed: " << result.error() << '\n';
        return 1;
    }

    if (handle.get_points_size() != 3 || handle.get_polys_size() == 0) {
        cleanup();
        std::cerr << "unexpected geometry: points=" << handle.get_points_size()
                  << " polys=" << handle.get_polys_size() << '\n';
        return 1;
    }

    cleanup();
    return 0;
}
