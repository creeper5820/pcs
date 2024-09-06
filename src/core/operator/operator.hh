#pragma once

#include "utility/single.hh"
#include <QVTKOpenGLNativeWidget.h>

namespace core::operators {
class Operators : public util::Singleton<Operators> {
public:
    Operators(util::Singleton<Operators>::token);
    ~Operators();
    Operators(const Operators&) = delete;
    Operators& operator=(const Operators&) = delete;

    void connectWidget(QVTKOpenGLNativeWidget* interface);

private:
    struct Impl;
    Impl* pimpl_;
};
};

namespace core {
using Operators = operators::Operators;
}