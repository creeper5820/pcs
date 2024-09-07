#include "renderer.hh"
#include "renderer.impl.hh"

using namespace core::renderer;
namespace speed = spdlog;

Renderer::Renderer(util::Singleton<Renderer>::token)
    : pimpl_(new Impl) {
    speed::info("load core::Renderer");
}
Renderer::~Renderer() {
    delete pimpl_;
    speed::info("delete core::Renderer");
}

/// Configuration
void Renderer::connectWidget(QVTKOpenGLNativeWidget* interface) {
    pimpl_->connectWidget(interface);
}

void Renderer::refresh() {
    pimpl_->refreshCamera();
}

void Renderer::render() {
    pimpl_->render();
}

void Renderer::removeObject(Object<vtkSmartPointer<vtkActor>>& object) {
    pimpl_->removeObject(object);
}
void Renderer::removeObject(Object<vtkSmartPointer<vtkTextActor>>& object) {
    pimpl_->removeObject(object);
}
void Renderer::removeObject(Object<vtkSmartPointer<vtkActor2D>>& object) {
    pimpl_->removeObject(object);
}
void Renderer::removeObject(Object<vtkSmartPointer<vtkAxesActor>>& object) {
    pimpl_->removeObject(object);
}

std::unique_ptr<CloudObject> Renderer::addCloud(const CloudBox& item, RenderColor color,
    double pointSize, double alpha) {
    return pimpl_->addCloud(item, color, pointSize, alpha);
}

FlatIndex Renderer::addText(const std::string& data, Eigen::Vector3d position,
    RenderColor color, int fontSize) {
    return pimpl_->addText(position, color, fontSize, data);
}

std::unique_ptr<PointObject> Renderer::addPoint(Eigen::Vector3d point, RenderColor color,
    double size, double alpha) {
    return pimpl_->addPoint(point, color, size, alpha);
}

StereoIndex Renderer::addLine(Eigen::Vector3d p1, Eigen::Vector3d p2, RenderColor color,
    double width, double alpha) {
    return pimpl_->addLine(p1, p2, color, width, alpha);
}

StereoIndex Renderer::addCoordinateSystem(Eigen::Vector3d center, double length,
    double width, double alpha) {
    return pimpl_->addCoordinateSystem(center, length, width, alpha);
}

void Renderer::removeStereoProps(StereoIndex index) {
    return pimpl_->removeStereoProps(index);
}

void Renderer::removeFlatProps(FlatIndex index) {
    return pimpl_->removeFlatProps(index);
}

void Renderer::removeAllProps() {
    return pimpl_->removeAllProps();
}

/// Modify property
void Renderer::modifyVisible(StereoIndex index, bool flag) {
    return pimpl_->modifyVisible(index, flag);
}

void Renderer::modifyPointSize(StereoIndex index, double size) {
    return pimpl_->modifyPointSize(index, size);
}

void Renderer::modifyColor(StereoIndex index, double r, double g, double b) {
    return pimpl_->modifyColor(index, r, g, b);
}

void Renderer::transformCloud(StereoIndex index, Eigen::Affine3d transform) {
    return pimpl_->transformCloud(index, transform);
}

void Renderer::setStereoPropsVisible(StereoIndex index, bool flag) {
    return pimpl_->setStereoPropsVisible(index, flag);
}

void Renderer::setFlatPropsVisible(FlatIndex index, bool flag) {
    return pimpl_->setFlatPropsVisible(index, flag);
}
