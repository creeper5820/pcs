#include "png-map.hh"

#include <algorithm>

#include <vtk/vtkCubeSource.h>
#include <vtk/vtkImageData.h>
#include <vtk/vtkPlaneSource.h>
#include <vtk/vtkPointData.h>
#include <vtk/vtkPolyDataMapper.h>
#include <vtk/vtkTexture.h>

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
    }
};

PngMapUnit::PngMapUnit(PngMapData const& map) noexcept
    : pimpl { std::make_unique<Impl>() } {
    initialize(map);
}

auto PngMapUnit::initialize(PngMapData const& map) noexcept -> void { pimpl->initialize(map); }

auto PngMapUnit::actor() noexcept -> SmartPointer<vtkActor> { return pimpl->actor; }

auto PngMapUnit::area_actor() noexcept -> SmartPointer<vtkActor> { return pimpl->area_actor; }

auto PngMapUnit::set_visibility(bool on) noexcept -> void {
    pimpl->actor->SetVisibility(on);
    pimpl->area_actor->SetVisibility(on);
}

PngMapUnit::~PngMapUnit() noexcept = default;

}
