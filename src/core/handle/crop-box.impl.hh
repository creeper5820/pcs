#pragma once

#include "core/handle/crop-box.hh"
#include "core/units/crop-box.hh"

using namespace pcs;

struct CropBoxHandle::Impl {
    std::unique_ptr<CropBoxUnit> unit = std::make_unique<CropBoxUnit>();
};
