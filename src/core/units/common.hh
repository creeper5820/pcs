#pragma once

#include <creeper-qt/utility/wrapper/pimpl.hh>

#include <vtk/vtkProperty.h>
#include <vtk/vtkSmartPointer.h>

namespace pcs {

template <typename T>
using SmartPointer = vtkSmartPointer<T>;

struct NormalUnit {
    auto set_visibility(this auto& self, bool on) {
        self.actor()->SetVisibility(on); //
    }
    auto set_pickable(this auto& self, bool on) {
        self.actor()->SetPickable(on); //
    }
    auto set_alpha(this auto& self, double alpha) {
        self.actor()->GetProperty()->SetOpacity(alpha);
    }
};

struct TripleDUnit {
    auto set_position(this auto& self, auto x, auto y, auto z) {
        self.actor()->SetPosition(x, y, z);
    }
    auto set_orientation(this auto& self, auto x, auto y, auto z) {
        self.actor()->SetOrientation(x, y, z);
    }
    auto set_color(this auto& self, auto r, auto g, auto b) {
        self.actor()->GetProperty()->SetColor(r, g, b);
    }

    auto get_position(this auto& self) { return self.actor()->GetPosition(); }
};

}
