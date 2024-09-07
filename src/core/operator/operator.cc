#include "operator.hh"
#include "core/pointcloud/cloud.hh"
#include "interactor.hh"

using namespace core::operators;

struct Operators::Impl {
public:
    void connectWidget(QVTKOpenGLNativeWidget* interface) {
        cloudManager_.connectWidget(interface);
        interface->interactor()->SetInteractorStyle(pickStyle_);
    }

private:
    CloudManager& cloudManager_ { CloudManager::instance() };
    vtkSmartPointer<PickStyle> pickStyle_ { vtkNew<PickStyle>() };
};

Operators::Operators(util::Singleton<Operators>::token)
    : pimpl_(new Impl) {
}

Operators::~Operators() {
    delete pimpl_;
}

void Operators::connectWidget(QVTKOpenGLNativeWidget* interface) {
    pimpl_->connectWidget(interface);
}
