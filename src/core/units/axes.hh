#pragma once
#include "common.hh"
#include "utility/pimpl.hh"
#include <vtkAxesActor.h>

namespace pcs {

struct AxesUnit : public NormalUnit, TripleDUnit {
    PCS_PIMPL_DEFINITION(AxesUnit)

public:
    auto actor() noexcept -> SmartPointer<vtkAxesActor>;
};

}
