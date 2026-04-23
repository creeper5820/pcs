#include "utility/morandi-pointcloud-color.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace pcs::utility {

namespace {

    constexpr auto kGoldenRatioConjugate = 0.6180339887498948;
    constexpr auto kPi                   = 3.14159265358979323846;

    auto clamp01(double value) noexcept -> double {
        return std::clamp(value, 0.0, 1.0);
    }

    auto hsv_to_rgb(double h, double s, double v) noexcept -> RgbColor {
        h = std::fmod(h, 1.0);
        if (h < 0.0) {
            h += 1.0;
        }

        s = clamp01(s);
        v = clamp01(v);

        const auto sector = h * 6.0;
        const auto i      = static_cast<int>(std::floor(sector));
        const auto f      = sector - static_cast<double>(i);

        const auto p = v * (1.0 - s);
        const auto q = v * (1.0 - s * f);
        const auto t = v * (1.0 - s * (1.0 - f));

        switch (i % 6) {
        case 0:
            return { v, t, p };
        case 1:
            return { q, v, p };
        case 2:
            return { p, v, t };
        case 3:
            return { p, q, v };
        case 4:
            return { t, p, v };
        default:
            return { v, p, q };
        }
    }

    auto srgb_to_linear(double c) noexcept -> double {
        c = clamp01(c);
        if (c <= 0.04045) {
            return c / 12.92;
        }
        return std::pow((c + 0.055) / 1.055, 2.4);
    }

    auto relative_luminance(RgbColor const& color) noexcept -> double {
        const auto r = srgb_to_linear(color.r);
        const auto g = srgb_to_linear(color.g);
        const auto b = srgb_to_linear(color.b);
        return 0.2126 * r + 0.7152 * g + 0.0722 * b;
    }

    auto ensure_black_contrast(RgbColor color) noexcept -> RgbColor {
        constexpr auto kTargetLuminance = 0.34;

        auto luminance = relative_luminance(color);
        if (luminance >= kTargetLuminance) {
            return color;
        }

        const auto lift = clamp01((kTargetLuminance - luminance) * 1.2);
        color.r         = clamp01(color.r + lift);
        color.g         = clamp01(color.g + lift * 0.95);
        color.b         = clamp01(color.b + lift * 0.9);
        return color;
    }

}

auto next_morandi_pointcloud_color() noexcept -> RgbColor {
    static auto index = std::size_t { 0 };

    const auto hue = std::fmod(0.14 + static_cast<double>(index) * kGoldenRatioConjugate, 1.0);
    const auto sat =
        clamp01(0.24 + 0.06 * std::sin((0.23 * static_cast<double>(index) + 0.17) * 2.0 * kPi));
    const auto val =
        clamp01(0.82 + 0.09 * std::sin((0.31 * static_cast<double>(index) + 0.53) * 2.0 * kPi));

    auto color = hsv_to_rgb(hue, sat, val);

    constexpr auto kGray = RgbColor { 0.66, 0.64, 0.62 };
    color.r              = clamp01(color.r * 0.72 + kGray.r * 0.28);
    color.g              = clamp01(color.g * 0.72 + kGray.g * 0.28);
    color.b              = clamp01(color.b * 0.72 + kGray.b * 0.28);

    ++index;
    return ensure_black_contrast(color);
}

}
