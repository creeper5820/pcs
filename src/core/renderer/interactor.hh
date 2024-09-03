#pragma once
#include <vtkAbstractPicker.h>
#include <vtkBuffer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRendererCollection.h>

class PickStyle : public vtkInteractorStyleTrackballCamera {
public:
    static PickStyle* New();
    vtkTypeMacro(PickStyle, vtkInteractorStyleTrackballCamera);

    void OnLeftButtonDown() override {
        auto pos = Interactor->GetEventPosition();
        auto renderer = Interactor->GetRenderWindow()->GetRenderers();
        auto picker = Interactor->GetPicker();

        picker->Pick(pos[0], pos[1], 0, renderer->GetFirstRenderer());
        picker->GetPickPosition(pickedPosition_);

        // Forward events.
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    inline void picked() {
    }

private:
    double pickedPosition_[3];
};
vtkStandardNewMacro(PickStyle);