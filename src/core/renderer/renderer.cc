#include "renderer.hh"
#include "renderer.impl.hh"

using namespace core::renderer;
namespace speed = spdlog;

Renderer::Renderer(util::Singleton<Renderer>::token)
    : pimpl_(new Impl) {
    speed::info("load Renderer");
}
Renderer::~Renderer() {
    delete pimpl_;
    speed::info("delete Renderer");
}

/// Configuration
void Renderer::connectWidget(QVTKOpenGLNativeWidget* interface) {
    pimpl_->connectWidget(interface);
}

void Renderer::resetCamera() {
    pimpl_->resetCamera();
}

void Renderer::render() {
    pimpl_->render();
}

template <vtkPropHandler Handler>
void Renderer::removeObject(Object<Handler>& object) {
    pimpl_->removeObject(object);
}

std::unique_ptr<CloudObject> Renderer::makeCloud(const CloudBox& item,
    RenderColor color, float pointSize) {
    return pimpl_->makeCloud(item, color, pointSize);
}

std::unique_ptr<PointObject> Renderer::makePoint(const Eigen::Vector3d& point,
    const RenderColor& color, float size) {
    return pimpl_->makePoint(point, color, size);
}

std::unique_ptr<TextObject> Renderer::makeText(const Eigen::Vector3d& position,
    const std::string& data, RenderColor color, int fontSize) {
    return pimpl_->makeText(position, fontSize, color, data);
}

std::unique_ptr<LineObject> Renderer::makeLine(const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2, const RenderColor& color, float width) {
    return pimpl_->makeLine(p1, p2, color, width);
}

std::unique_ptr<CoordinateObject> Renderer::makeCoordinate(const Eigen::Vector3d& center,
    double length, double width) {
    return pimpl_->makeCoordinate(center, length, width);
}

std::unique_ptr<CubeObject> Renderer::makeCube(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
    const RenderColor& color, double alpha) {
    return pimpl_->makeCube(p1, p2, color, alpha);
}