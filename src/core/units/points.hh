#pragma once
#include "common.hh"
#include "core/renderer.hh"

#include <vtk/vtkActor.h>
#include <vtk/vtkPoints.h>
#include <vtk/vtkProp.h>

namespace pcs {

namespace details::unit::points {

    template <class T>
    concept point_struct_trait = requires(const T& t) {
        { auto { t.x } } -> std::floating_point;
        { auto { t.y } } -> std::floating_point;
        { auto { t.z } } -> std::floating_point;
    };
    template <class T>
    concept point_wrapper_trait = requires(const T& t) {
        { auto { t.x() } } -> std::floating_point;
        { auto { t.y() } } -> std::floating_point;
        { auto { t.z() } } -> std::floating_point;
    };

    template <class T>
    concept point_trait = point_struct_trait<T> || point_wrapper_trait<T>;

    template <class T>
    concept points_trait = requires(const T& t) {
        requires std::ranges::range<T>;
        requires point_trait<decltype(auto { t[0] })>;
    };

    template <class T>
    struct PointAdapter final {

        std::conditional_t<std::is_const_v<T>, const T&, T&> point;

        constexpr explicit PointAdapter(T& point)
            requires point_trait<T>
            : point { point } { }

        auto x(this auto&& self) {
            if constexpr (point_struct_trait<T>) {
                return std::forward<decltype(self)>(self).point.x;
            }
            if constexpr (point_wrapper_trait<T>) {
                return std::forward<decltype(self)>(self).point.x();
            }
        }
        auto y(this auto&& self) {
            if constexpr (point_struct_trait<T>) {
                return std::forward<decltype(self)>(self).point.y;
            }
            if constexpr (point_wrapper_trait<T>) {
                return std::forward<decltype(self)>(self).point.y();
            }
        }
        auto z(this auto&& self) {
            if constexpr (point_struct_trait<T>) {
                return std::forward<decltype(self)>(self).point.z;
            }
            if constexpr (point_wrapper_trait<T>) {
                return std::forward<decltype(self)>(self).point.z();
            }
        }

        auto get_information(this auto const& self) {
            return std::format("Point[ x={}, y={}, z={} ]", self.x(), self.y(), self.z());
        }
    };
}

struct PointsUnit : public NormalUnit, TripleDUnit {
public:
    ~PointsUnit() noexcept;

    PointsUnit(const PointsUnit&)            = delete;
    PointsUnit& operator=(const PointsUnit&) = delete;

    template <class Points>
        requires details::unit::points::points_trait<Points>
    explicit PointsUnit(Points const& points) noexcept
        : pcs::PointsUnit { } {
        using namespace details::unit::points;

        auto vtk_points = vtkPoints::New();
        for (auto const& point : points) {
            const auto p = PointAdapter { point };
            vtk_points->InsertNextPoint(p.x(), p.y(), p.z());
        }
        initialize(vtk_points);
    }

    auto actor() noexcept -> SmartPointer<vtkActor>;
    auto actor() const noexcept -> SmartPointer<vtkActor>;

    auto attach(Renderer&) noexcept -> void;
    auto detach(Renderer&) noexcept -> void;
    auto pick(Renderer&, int display_x, int display_y) const noexcept
        -> std::optional<Renderer::Position>;

    auto coordinate_actor() noexcept -> SmartPointer<vtkProp>;
    auto coordinate_actor() const noexcept -> SmartPointer<vtkProp>;
    auto coordinate_origin_actor() noexcept -> SmartPointer<vtkProp>;
    auto coordinate_origin_actor() const noexcept -> SmartPointer<vtkProp>;

    auto set_coordinate_visibility(bool on) noexcept -> void;
    auto set_coordinate_position(double x, double y, double z) noexcept -> void;
    auto get_points_size() const noexcept -> std::size_t;

private:
    explicit PointsUnit() noexcept;

    auto initialize(SmartPointer<vtkPoints>) noexcept -> void;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}
