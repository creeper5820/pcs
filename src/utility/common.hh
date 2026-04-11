#pragma once

#include <creeper-qt/utility/trait/widget.hh>

#include <qpointer.h>
#include <qwidget.h>

template <typename T>
using raw_pointer = T*;

template <creeper::widget_trait T>
struct Component {
    QPointer<T> component;

    // --- Conversion Operators ---

    // Conversion to QPointer<QWidget> (assuming T inherits QWidget)
    operator QPointer<QWidget>() const noexcept { return component; }

    // --- Access Operators ---

    // Dereference operator (allows *component_instance)
    T& operator*() const noexcept {
        // QPointer implicitly converts to T*
        return *component;
    }

    // Member access operator (allows component_instance->method())
    T* operator->() const noexcept {
        // QPointer implicitly converts to T*
        return component;
    }

    // --- Forwarding Constructor ---

    template <typename... Args>
    explicit Component(Args... args) noexcept
        requires std::constructible_from<T, Args...>
        : component { new T { std::forward<Args>(args)... } } { }
};
