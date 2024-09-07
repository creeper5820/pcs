#pragma once
#include <Eigen/Dense>

#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

#include <functional>

using RenderColor = std::tuple<double, double, double>;

#define EnableObjectHashCompare(Object)                 \
    template <>                                         \
    struct std::hash<Object> {                          \
        std::size_t operator()(const Object& f) const { \
            return std::hash<void*> {}(f.hashValue());  \
        }                                               \
    };

namespace core::renderer {
inline std::function<void(vtkProp*)> objectKiller([](vtkProp*) { });

template <typename Handler>
concept vtkPropHandler = requires(Handler handler) {
    dynamic_cast<vtkProp*>(handler.Get());
};

template <vtkPropHandler Handler>
class Object {
public:
    Object(Handler handler)
        : handler_(handler) {
    }
    ~Object() {
        objectKiller(handler_);
    }
    Object(const Object&) = delete;
    Object& operator=(const Object&) = delete;

    inline void setVisible(bool flag) {
        handler_->SetVisibility(flag);
    }
    inline void setPickable(bool flag) {
        handler_->SetPickable(flag);
    }
    inline Handler handler() const {
        return handler_;
    }

public:
    inline bool operator==(const Object& r) const {
        return handler_ == r.handler_;
    }
    inline void* hashValue() const {
        return handler_.Get();
    }

protected:
    Handler handler_;
};

template <typename Handler>
class Object3D : public Object<Handler> {
public:
    Object3D(Handler handler)
        : Object<Handler>(handler) {
    }
    inline void setPosition(double x, double y, double z) {
        handler_->SetPosition(x, y, z);
    }
    inline void setOrientation(double x, double y, double z) {
        handler_->SetOrientation(x, y, z);
    }

protected:
    using Object<Handler>::handler_;
};

template <typename Handler>
class Object2D : public Object<Handler> {
public:
    Object2D(vtkProp* handler)
        : Object<Handler>(handler) {
    }
    inline void setDraggable(bool flag) {
        handler_->SetDragable(flag);
    }

protected:
    using Object<Handler>::handler_;
};

using vtkActorHandler = vtkSmartPointer<vtkActor>;
class PointObject final : public Object3D<vtkActorHandler> {
public:
    PointObject(vtkActorHandler handler)
        : Object3D(handler) {
    }
    inline void setPointSize(double size) {
        handler_->GetProperty()->SetPointSize(size);
    }
    inline void setColor(double r, double g, double b) {
        handler_->GetProperty()->SetColor(r, g, b);
    }
};
class CloudObject final : public Object3D<vtkActorHandler> {
public:
    CloudObject(vtkActorHandler handler)
        : Object3D(handler) {
    }
    inline void setPointSize(double size) {
        handler_->GetProperty()->SetPointSize(size);
    }
    inline void setColor(double r, double g, double b) {
        handler_->GetProperty()->SetColor(r, g, b);
    }
};
}

using PointObject = core::renderer::PointObject;
using CloudObject = core::renderer::CloudObject;

EnableObjectHashCompare(PointObject);
EnableObjectHashCompare(CloudObject);
