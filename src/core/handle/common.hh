#pragma once

namespace pcs {

struct NormalUnit {
    auto set_visibility(bool on) noexcept -> void;
    auto set_pickable(bool on) noexcept -> void;
    auto set_alpha(double alpha) noexcept -> void;
};

}
