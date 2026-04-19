#include "png-map.hh"

#include "core/map/png-map-transform.hh"

#include <algorithm>

#include <vtk/vtkAxesActor.h>
#include <vtk/vtkCubeSource.h>
#include <vtk/vtkImageData.h>
#include <vtk/vtkProperty.h>
#include <vtk/vtkPlaneSource.h>
#include <vtk/vtkPointData.h>
#include <vtk/vtkPolyDataMapper.h>
#include <vtk/vtkSphereSource.h>
#include <vtk/vtkTexture.h>
#include <vtk/vtkTransform.h>

namespace pcs {

struct PngMapUnit::Impl {
    SmartPointer<vtkImageData> image;
    SmartPointer<vtkTexture> texture;
    SmartPointer<vtkPlaneSource> plane;
    SmartPointer<vtkCubeSource> area_box;
    SmartPointer<vtkPolyDataMapper> mapper;
    SmartPointer<vtkPolyDataMapper> area_mapper;
    SmartPointer<vtkActor> actor;
    SmartPointer<vtkActor> area_actor;
    SmartPointer<vtkAxesActor> frame_axes;
    SmartPointer<vtkTransform> frame_transform;
    SmartPointer<vtkSphereSource> frame_origin;
    SmartPointer<vtkPolyDataMapper> frame_origin_mapper;
    SmartPointer<vtkActor> frame_origin_actor;

    auto update_pixels(std::vector<std::uint8_t> const& pixels) -> bool {
        if (image == nullptr) {
            return false;
        }

        auto dimensions   = image->GetDimensions();
        const auto width  = static_cast<std::size_t>(dimensions[0]);
        const auto height = static_cast<std::size_t>(dimensions[1]);
        if (pixels.size() != width * height) {
            return false;
        }

        for (std::size_t y = 0; y < height; ++y) {
            for (std::size_t x = 0; x < width; ++x) {
                auto* pixel = static_cast<unsigned char*>(
                    image->GetScalarPointer(static_cast<int>(x), static_cast<int>(y), 0));
                pixel[0] = pixels[y * width + x];
            }
        }

        image->Modified();
        texture->Modified();
        return true;
    }

    auto initialize(PngMapData const& map) {
        image = vtkImageData::New();
        image->SetDimensions(static_cast<int>(map.width), static_cast<int>(map.height), 1);
        image->AllocateScalars(VTK_UNSIGNED_CHAR, 1);

        for (std::size_t y = 0; y < map.height; ++y) {
            for (std::size_t x = 0; x < map.width; ++x) {
                auto* pixel = static_cast<unsigned char*>(
                    image->GetScalarPointer(static_cast<int>(x), static_cast<int>(y), 0));
                pixel[0] = map.pixels[y * map.width + x];
            }
        }

        texture = vtkTexture::New();
        texture->SetInputData(image);
        texture->InterpolateOff();

        const auto max_x = map.origin_x + map.resolution * static_cast<double>(map.width);
        const auto max_y = map.origin_y + map.resolution * static_cast<double>(map.height);

        plane = vtkPlaneSource::New();
        plane->SetOrigin(map.origin_x, map.origin_y, map.plane_z);
        plane->SetPoint1(max_x, map.origin_y, map.plane_z);
        plane->SetPoint2(map.origin_x, max_y, map.plane_z);
        plane->Update();

        mapper = vtkPolyDataMapper::New();
        mapper->SetInputConnection(plane->GetOutputPort());

        actor = vtkActor::New();
        actor->SetMapper(mapper);
        actor->SetTexture(texture);
        actor->GetProperty()->LightingOff();

        const auto z_min = std::min(map.z_area_start, map.z_area_end);
        const auto z_max = std::max(map.z_area_start, map.z_area_end);

        area_box = vtkCubeSource::New();
        area_box->SetBounds(map.origin_x, max_x, map.origin_y, max_y, z_min, z_max);
        area_box->Update();

        area_mapper = vtkPolyDataMapper::New();
        area_mapper->SetInputConnection(area_box->GetOutputPort());

        area_actor = vtkActor::New();
        area_actor->SetMapper(area_mapper);
        area_actor->GetProperty()->SetRepresentationToWireframe();
        area_actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
        area_actor->GetProperty()->SetLineWidth(2.0);
        area_actor->GetProperty()->LightingOff();
        area_actor->PickableOff();

        frame_axes = vtkAxesActor::New();
        frame_axes->AxisLabelsOff();
        frame_axes->PickableOff();
        frame_axes->GetXAxisShaftProperty()->SetLineWidth(4.0);
        frame_axes->GetYAxisShaftProperty()->SetLineWidth(4.0);
        frame_axes->GetZAxisShaftProperty()->SetLineWidth(4.0);
        frame_axes->GetXAxisTipProperty()->SetColor(1.0, 0.42, 0.32);
        frame_axes->GetYAxisTipProperty()->SetColor(0.22, 0.76, 0.47);
        frame_axes->GetZAxisTipProperty()->SetColor(0.32, 0.62, 1.0);

        frame_transform = vtkTransform::New();
        frame_axes->SetUserTransform(frame_transform);

        frame_origin = vtkSphereSource::New();
        frame_origin_mapper = vtkPolyDataMapper::New();
        frame_origin_mapper->SetInputConnection(frame_origin->GetOutputPort());

        frame_origin_actor = vtkActor::New();
        frame_origin_actor->SetMapper(frame_origin_mapper);
        frame_origin_actor->PickableOff();
        frame_origin_actor->GetProperty()->LightingOff();
        frame_origin_actor->GetProperty()->SetColor(1.0, 0.85, 0.1);
        update_frame(map);
    }

    auto update_frame(PngMapData const& map) -> void {
        if (frame_axes == nullptr) {
            return;
        }

        const auto view   = make_png_map_transform_view(map);
        const auto anchor = png_map_anchor_world(view);
        const auto span_x = static_cast<double>(map.width) * map.resolution;
        const auto span_y = static_cast<double>(map.height) * map.resolution;
        const auto length = std::clamp(std::min(span_x, span_y) * 0.15, 0.2, 5.0);
        const auto marker_radius = std::clamp(length * 0.05, 0.02, 0.2);
        constexpr auto kFrameZOffset = 1e-3;

        frame_axes->SetTotalLength(length, length, length * 0.6);
        frame_transform->Identity();
        frame_transform->Translate(anchor[0], anchor[1], map.plane_z + kFrameZOffset);
        frame_transform->RotateZ(map.frame_config.yaw_deg);
        frame_transform->Modified();

        frame_origin->SetCenter(anchor[0], anchor[1], map.plane_z + kFrameZOffset);
        frame_origin->SetRadius(marker_radius);
        frame_origin->SetThetaResolution(24);
        frame_origin->SetPhiResolution(24);
        frame_origin->Update();
        frame_origin_actor->Modified();
        frame_axes->Modified();
    }
};

PngMapUnit::PngMapUnit(PngMapData const& map) noexcept
    : pimpl { std::make_unique<Impl>() } {
    initialize(map);
}

auto PngMapUnit::initialize(PngMapData const& map) noexcept -> void { pimpl->initialize(map); }

auto PngMapUnit::actor() noexcept -> SmartPointer<vtkActor> { return pimpl->actor; }

auto PngMapUnit::area_actor() noexcept -> SmartPointer<vtkActor> { return pimpl->area_actor; }

auto PngMapUnit::frame_actor() noexcept -> vtkProp* { return pimpl->frame_axes; }

auto PngMapUnit::frame_origin_actor() noexcept -> SmartPointer<vtkActor> {
    return pimpl->frame_origin_actor;
}

auto PngMapUnit::actor() const noexcept -> SmartPointer<vtkActor> { return pimpl->actor; }

auto PngMapUnit::area_actor() const noexcept -> SmartPointer<vtkActor> { return pimpl->area_actor; }

auto PngMapUnit::frame_actor() const noexcept -> vtkProp* { return pimpl->frame_axes; }

auto PngMapUnit::frame_origin_actor() const noexcept -> SmartPointer<vtkActor> {
    return pimpl->frame_origin_actor;
}

auto PngMapUnit::set_visibility(bool on) noexcept -> void {
    pimpl->actor->SetVisibility(on);
    pimpl->area_actor->SetVisibility(on);
    pimpl->frame_axes->SetVisibility(on);
    pimpl->frame_origin_actor->SetVisibility(on);
}

auto PngMapUnit::update_pixels(std::vector<std::uint8_t> const& pixels) noexcept -> bool {
    return pimpl->update_pixels(pixels);
}

auto PngMapUnit::update_frame_config(PngMapData const& map) noexcept -> void {
    pimpl->update_frame(map);
}

PngMapUnit::~PngMapUnit() noexcept = default;

}
