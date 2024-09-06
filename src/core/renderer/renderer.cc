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

/// CRUD
int Renderer::addCloud(const PointCloud& item) {
    return pimpl_->addCloud(item);
}
void Renderer::removeCloud(int index) {
    pimpl_->removeCloud(index);
}
void Renderer::removeAllCloud() {
    pimpl_->removeAllCloud();
}

/// Modify property
void Renderer::modifyVisible(int index, bool flag) {
    pimpl_->modifyVisible(index, flag);
}
void Renderer::modifyPointSize(int index, double size) {
    pimpl_->modifyPointSize(index, size);
}
void Renderer::modifyColor(int index, double r, double g, double b) {
    pimpl_->modifyColor(index, r, g, b);
}
void Renderer::transformCloud(int index, Eigen::Affine3d transform) {
    pimpl_->transformCloud(index, transform);
}

/// Visualization
int Renderer::addText(const TextConfig& config) {
    return pimpl_->addText(config);
}
int Renderer::addPoint(Eigen::Vector3d point, RGB color,
    double size, double alpha) {
    return pimpl_->addPoint(point, color, size, alpha);
}
int Renderer::addLine(Eigen::Vector3d p1, Eigen::Vector3d p2,
    RGB color, double width, double alpha) {
    return pimpl_->addLine(p1, p2, color, width, alpha);
}
int Renderer::addCoordinateSystem(Eigen::Vector3d center,
    double length, double width, double alpha) {
    return pimpl_->addCoordinateSystem(center, length, width, alpha);
}

void Renderer::removeText(int index) {
    pimpl_->removeFlatProps(index);
}
void Renderer::removePoint(int index) {
    pimpl_->removeStereoProps(index);
}
void Renderer::removeLine(int index) {
    pimpl_->removeStereoProps(index);
}
void Renderer::removeCoordinateSystem(int index) {
    pimpl_->removeStereoProps(index);
}

void Renderer::setStereoPropsVisible(int index, bool flag) {
    pimpl_->setStereoPropsVisible(index, flag);
}
void Renderer::setFlatPropsVisible(int index, bool flag) {
    pimpl_->setFlatPropsVisible(index, flag);
}
