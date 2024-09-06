#pragma once
#include <string>
#include <tuple>

namespace core::renderer {
using RenderColor = std::tuple<double, double, double>;
using Translation = std::tuple<double, double, double>;

struct Property {
    bool visible = true;
    bool pickable = true;
    RenderColor color;
    Translation pos;
};

struct PointCloudProperty : Property {
    double size;
    double alpha;
};

struct PointProperty : Property {
    double size;
    double alpha;
};

struct LineProperty : Property {
    Translation pos2;
    double width;
    double alpha;
};

struct TextProperty : Property {
    std::string data;
    int fontSize;
};

struct CoordinateProperty : Property {
    double length;
    double width;
    double alpha;
};
}
