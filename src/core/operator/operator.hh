#pragma once

#include "utility/single.hh"
#include <QVTKOpenGLNativeWidget.h>

namespace core::operators {
class Operators : public util::Singleton<Operators> {
public:
    explicit Operators(util::Singleton<Operators>::token);
    ~Operators();
    Operators(const Operators&) = delete;
    Operators& operator=(const Operators&) = delete;

    void connectWidget(QVTKOpenGLNativeWidget* interface);

    void useNormalStyle();
    void usePointPicker();

private:
    struct Impl;
    Impl* pimpl_;
};
};

using Operators = core::operators::Operators;
