#pragma once
#include <functional>

#define EnableHashCompare(Key)                       \
    template <>                                      \
    struct std::hash<Key> {                          \
        std::size_t operator()(const Key& f) const { \
            return std::hash<int> {}(f.index);       \
        }                                            \
    }

namespace core::renderer {
template <typename T>
struct Index {
    T& operator++() {
        this->index++;
        return *reinterpret_cast<T*>(this);
    }
    T operator++(int) {
        auto temp = *this;
        this->index++;
        return *reinterpret_cast<T*>(&temp);
    }
    bool operator==(const Index& r) const {
        return index == r.index;
    }
    bool valid() const {
        return index != -1;
    }
    int index = -1;
};
struct FlatIndex final : Index<FlatIndex> { };
struct StereoIndex final : Index<StereoIndex> { };
}

EnableHashCompare(core::renderer::FlatIndex);
EnableHashCompare(core::renderer::StereoIndex);

using FlatIndex = core::renderer::FlatIndex;
using StereoIndex = core::renderer::StereoIndex;