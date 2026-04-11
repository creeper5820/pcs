#include "axes.hh"

#include <vtk/vtkAxesActor.h>
#include <vtk/vtkCaptionActor2D.h>
#include <vtk/vtkTextProperty.h>

using namespace pcs;

struct AxesUnit::Impl {
    SmartPointer<vtkAxesActor> actor;

    explicit Impl() noexcept { actor = vtkAxesActor::New(); }

    auto set_total_length(double r, double g, double b) noexcept { actor->SetTotalLength(r, g, b); }

    ///
    /// Axis: 'x', 'y', 'z'
    ///

    auto set_axis_color(char axis, double r, double g, double b) noexcept {
        /*  */ if (axis == 'x') {
            actor->GetXAxisShaftProperty()->SetColor(r, g, b);
        } else if (axis == 'y') {
            actor->GetYAxisShaftProperty()->SetColor(r, g, b);
        } else if (axis == 'z') {
            actor->GetZAxisShaftProperty()->SetColor(r, g, b);
        }
    }
    auto set_axis_label_color(char axis, double r, double g, double b) noexcept {
        /*  */ if (axis == 'x') {
            actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(r, g, b);
        } else if (axis == 'y') {
            actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(r, g, b);
        } else if (axis == 'z') {
            actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(r, g, b);
        }
    }
    auto set_axis_line_width(char axis, double w) noexcept {
        /*  */ if (axis == 'x') {
            actor->GetXAxisShaftProperty()->SetLineWidth(w);
        } else if (axis == 'y') {
            actor->GetYAxisShaftProperty()->SetLineWidth(w);
        } else if (axis == 'z') {
            actor->GetZAxisShaftProperty()->SetLineWidth(w);
        }
    }
};

auto AxesUnit::actor() noexcept -> SmartPointer<vtkAxesActor> { return pimpl->actor; }

AxesUnit::AxesUnit() noexcept
    : pimpl { std::make_unique<Impl>() } { }

AxesUnit::~AxesUnit() noexcept = default;
