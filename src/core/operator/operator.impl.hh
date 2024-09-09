#pragma once

#include "core/operator/interactor/normal.hh"
#include "core/operator/interactor/point-picker.hh"
#include "core/operator/operator.hh"
#include "core/pointcloud/cloud.hh"

namespace core::operators {

struct Operators::Impl {
public:
    void connectWidget(QVTKOpenGLNativeWidget* interface) {
        pointPicker_ = vtkNew<Selector>();
        normalStyle_ = vtkNew<NormalStyle>();
        cloudManager_.connectWidget(interface);
        interactor_ = interface->interactor();

        usePointPicker();
    }

    void setInteractor(vtkInteractorObserver* interactor) {
        interactor_->SetInteractorStyle(interactor);
    }

    void usePointPicker() {
        interactor_->SetInteractorStyle(pointPicker_);
    }
    void useNormalStyle() {
        interactor_->SetInteractorStyle(normalStyle_);
    }

private:
    QVTKInteractor* interactor_;
    CloudManager& cloudManager_ = CloudManager::instance();

    // Preset
    vtkSmartPointer<Selector> pointPicker_;
    vtkSmartPointer<NormalStyle> normalStyle_;
};
}