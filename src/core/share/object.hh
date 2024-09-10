#pragma once
#include <Eigen/Dense>

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

#include <functional>
#include <utility>

#define EnableObjectHashCompare(Object)                 \
    template <>                                         \
    struct std::hash<Object> {                          \
        std::size_t operator()(const Object& f) const { \
            return std::hash<void*> {}(f.hashValue());  \
        }                                               \
    };

namespace core::renderer {
inline std::function<void(vtkProp*)>
    objectKiller([](vtkProp*) { });

template <typename Handler>
concept vtkPropHandler = requires(Handler handler) {
    dynamic_cast<vtkProp*>(handler.Get());
};
template <vtkPropHandler Handler>
class Object {
public:
    explicit Object(Handler handler)
        : handler_(std::move(handler)) {
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
    inline void setAlpha(double alpha) {
        handler_->GetProperty()->SetOpacity(alpha);
    }

public:
    inline bool operator==(const Object& r) const {
        return handler_ == r.handler_;
    }
    inline void* hashValue() const {
        return handler_.Get();
    }
    inline Handler handler() const {
        return handler_;
    }

protected:
    Handler handler_;
};

template <vtkPropHandler Handler>
class Object3D : public Object<Handler> {
public:
    explicit Object3D(Handler handler)
        : Object<Handler>(handler) {
    }
    inline void setPosition(double x, double y, double z) {
        handler_->SetPosition(x, y, z);
    }
    inline void setOrientation(double x, double y, double z) {
        handler_->SetOrientation(x, y, z);
    }

public:
    inline Eigen::Vector3d getPosition() {
        const auto position = handler_->GetPosition();
        return { position[0], position[1], position[2] };
    }
    inline Eigen::Vector3d getOrientation() {
        auto rotation = handler_->GetOrientation();
        return { rotation[0], rotation[1], rotation[2] };
    }

protected:
    using Object<Handler>::handler_;
};

template <vtkPropHandler Handler>
class Object2D : public Object<Handler> {
public:
    explicit Object2D(Handler handler)
        : Object<Handler>(handler) {
    }
    inline void setDraggable(bool flag) {
        handler_->SetDragable(flag);
    }

protected:
    using Object<Handler>::handler_;
};

/// vtkActor
using vtkActorHandler = vtkSmartPointer<vtkActor>;
class PointObject final : public Object3D<vtkActorHandler> {
public:
    explicit PointObject(vtkActorHandler handler)
        : Object3D(std::move(handler)) {
    }
    inline void setPointSize(float size) {
        handler_->GetProperty()->SetPointSize(size);
    }
    inline void setColor(double r, double g, double b) {
        handler_->GetProperty()->SetColor(r, g, b);
    }
};
class CloudObject final : public Object3D<vtkActorHandler> {
public:
    explicit CloudObject(vtkActorHandler handler)
        : Object3D(std::move(handler)) {
    }
    inline void setPointSize(float size) {
        handler_->GetProperty()->SetPointSize(size);
    }
    inline void setColor(double r, double g, double b) {
        handler_->GetProperty()->SetColor(r, g, b);
    }
};
class LineObject final : public Object3D<vtkActorHandler> {
public:
    explicit LineObject(vtkActorHandler handler)
        : Object3D(std::move(handler)) {
    }
    inline void setColor(double r, double g, double b) {
        handler_->GetProperty()->SetColor(r, g, b);
    }
    inline void setLineWidth(float width) {
        handler_->GetProperty()->SetLineWidth(width);
    }

public:
    inline double getLength() {
        return handler_->GetLength();
    }
};
class CubeObject final : public Object3D<vtkActorHandler> {
public:
    explicit CubeObject(vtkActorHandler handler)
        : Object3D(std::move(handler)) {
    }
    inline void setColor(double r, double g, double b) {
        handler_->GetProperty()->SetColor(r, g, b);
    }
};

/// vtkTextActor
using vtkTextHandler = vtkSmartPointer<vtkTextActor>;
class TextObject final : public Object2D<vtkTextHandler> {
public:
    explicit TextObject(vtkTextHandler handler)
        : Object2D(std::move(handler)) {
    }
    inline void setFontSize(int size) {
        handler_->GetTextProperty()->SetFontSize(size);
    }
    inline void setColor(double r, double g, double b) {
        handler_->GetTextProperty()->SetColor(r, g, b);
    }
    inline void setPosition(double x, double y) {
        handler_->SetPosition(x, y);
    }
    inline void setText(const std::string& text) {
        handler_->SetInput(text.c_str());
    }
};

/// vtkAxesActor
using vtkAxesHandler = vtkSmartPointer<vtkAxesActor>;
class CoordinateObject final : public Object3D<vtkAxesHandler> {
public:
    enum class AxesShaft : uint8_t {
        CYLINDER_SHAFT = vtkAxesActor::CYLINDER_SHAFT,
        LINE_SHAFT = vtkAxesActor::LINE_SHAFT,
        CUSTOM_SHAFT = vtkAxesActor::USER_DEFINED_SHAFT
    };
    explicit CoordinateObject(vtkAxesHandler handler)
        : Object3D(std::move(handler)) {
    }
    inline void setConeRadius(double radius) {
        handler_->SetConeRadius(radius);
    }
    inline void setConeResolution(int resolution) {
        handler_->SetConeResolution(resolution);
    }
    inline void setShaftType(AxesShaft type) {
        handler_->SetShaftType(static_cast<uint8_t>(type));
    }
};
}

/// vtkActor
using PointObject = core::renderer::PointObject;
EnableObjectHashCompare(PointObject);

using CloudObject = core::renderer::CloudObject;
EnableObjectHashCompare(CloudObject);

using LineObject = core::renderer::LineObject;
EnableObjectHashCompare(LineObject);

using CubeObject = core::renderer::CubeObject;
EnableObjectHashCompare(CubeObject);

/// vtkTextActor
using TextObject = core::renderer::TextObject;
EnableObjectHashCompare(TextObject);

/// vtkAxesActor
using CoordinateObject = core::renderer::CoordinateObject;
EnableObjectHashCompare(CoordinateObject);
