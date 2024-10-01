#pragma once

namespace util {

template <typename T>
struct Range {
public:
    T min;
    T max;
    constexpr bool operator()(T value) const {
        return value >= min && value <= max;
    }
    constexpr bool contains(T value) const {
        return value >= min && value <= max;
    }
    constexpr T mid() const {
        return (min + max) / T(2);
    }
    constexpr T length() const {
        return max - min;
    }
    constexpr bool valid() const {
        return min < max;
    }
};

}