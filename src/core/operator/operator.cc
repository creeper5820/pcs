#include "operator.hh"
#include "operator.impl.hh"

using namespace core::operators;

Operators::Operators(util::Singleton<Operators>::token)
    : pimpl_(new Impl) {
}

Operators::~Operators() {
    delete pimpl_;
}

void Operators::connectWidget(QVTKOpenGLNativeWidget* interface) {
    pimpl_->connectWidget(interface);
}

void Operators::useNormalStyle() {
    pimpl_->useNormalStyle();
}
void Operators::usePointPicker() {
    pimpl_->usePointPicker();
}
