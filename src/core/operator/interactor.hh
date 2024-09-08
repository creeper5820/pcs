#pragma once

#include "core/pointcloud/cloud.hh"
#include "core/renderer/renderer.hh"

#include <vtkAbstractPicker.h>
#include <vtkBuffer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRendererCollection.h>

#include <Eigen/Core>
#include <spdlog/spdlog.h>

namespace core::operators {
// TODO: One day we will fuck it up.
inline Eigen::Vector3d clickedPoint { 0, 0, 0 };
inline bool isClicked { false };
}

class PickStyle : public vtkInteractorStyleTrackballCamera {
public:
    static PickStyle* New();
    vtkTypeMacro(PickStyle, vtkInteractorStyleTrackballCamera);

    void OnLeftButtonDown() override {
        mousePosition_ = { this->Interactor->GetEventPosition()[0],
            this->Interactor->GetEventPosition()[1] };

        // Forward events.
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    void OnLeftButtonUp() override {
        auto pos = Interactor->GetEventPosition();
        if (pos[0] == mousePosition_[0] && pos[1] == mousePosition_[1]) {
            auto renderer = Interactor->GetRenderWindow()->GetRenderers();
            auto picker = Interactor->GetPicker();

            auto flag = picker->Pick(pos[0], pos[1], 0, renderer->GetFirstRenderer());
            auto position = picker->GetPickPosition();

            auto& isClicked = core::operators::isClicked;
            auto& clickedPoint = core::operators::clickedPoint;
            if (flag != 0) {
                showClickedPoint({ position[0], position[1], position[2] });
                clickedPoint = { position[0], position[1], position[2] };
                isClicked = true;
            }
        }

        // Forward events.
        vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
    }

private:
    Eigen::Vector2d mousePosition_ = { 0, 0 };

    CloudManager& cloudManager_ = CloudManager::instance();
    Renderer& renderer_ = Renderer::instance();

    void showClickedPoint(Eigen::Vector3d position) {
        static auto pointObject = renderer_.makePoint(
            position, Renderer::miku, 10);

        pointObject = renderer_.makePoint(
            position, Renderer::miku, 10);
    }
};
inline vtkStandardNewMacro(PickStyle);
