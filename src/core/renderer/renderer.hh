#pragma once
#include "core/cloud/item.hh"
#include "utility/single.hh"

#include <QVTKOpenGLNativeWidget.h>

namespace core::renderer {

class Renderer : public util::Singleton<Renderer> {
public:
    using PointCloud = core::cloud::Item;
    using RGB = std::tuple<double, double, double>;

    struct TextConfig {
        std::string data;
        Eigen::Vector2d position;
        int fontSize;
        RGB color;
    };

    Renderer(util::Singleton<Renderer>::token);
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    ~Renderer();

    /// Configuration
    void connectWidget(QVTKOpenGLNativeWidget* interface);
    void refresh();

    /// CRUD
    int addCloud(const PointCloud& item);
    void removeCloud(int index);
    void removeAllCloud();

    /// Modify property
    void modifyVisible(int index, bool flag);
    void modifyPointSize(int index, double size);
    void modifyColor(int index, double r, double g, double b);
    void transformCloud(int index, Eigen::Affine3d transform);

    /// Visualization
    int addText(const TextConfig& text);
    int addPoint(Eigen::Vector3d point, RGB color, double size,
        double alpha = 1.0);
    int addLine(Eigen::Vector3d p1, Eigen::Vector3d p2, RGB color,
        double width, double alpha = 1.0);
    int addCoordinateSystem(Eigen::Vector3d center, double length,
        double width, double alpha = 1.0);

    void removeText(int index);
    void removePoint(int index);
    void removeLine(int index);
    void removeCoordinateSystem(int index);

    void setStereoPropsVisible(int index, bool flag);
    void setFlatPropsVisible(int index, bool flag);

private:
    struct Impl;
    Impl* pimpl_;
};

};

namespace core {
using Renderer = renderer::Renderer;
}
