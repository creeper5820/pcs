#pragma once
#include "core/share/object.hh"

namespace core::renderer {

class CubeObjects {
public:
    CubeObjects() = default;
    CubeObjects(const CubeObjects&) = delete;
    CubeObjects& operator=(const CubeObjects&) = delete;

    inline void pushBack(std::unique_ptr<CubeObject> object) {
        objects_.push_back(std::move(object));
    }
    inline void setColor(double r, double g, double b) {
        for (auto& object : objects_)
            object->setColor(r, g, b);
    }

private:
    std::vector<std::unique_ptr<CubeObject>> objects_;
};

}