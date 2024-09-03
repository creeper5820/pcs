#include "core/renderer/renderer.hh"
#include "core/renderer/interactor.hh"

#include <vtkAbstractPicker.h>
#include <vtkBuffer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRendererCollection.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkVertexGlyphFilter.h>

#include <spdlog/spdlog.h>

using namespace core::renderer;
namespace speed = spdlog;

struct Renderer::Impl {
public:
    explicit Impl() = default;

    void connectWidget(QVTKOpenGLNativeWidget* interface) {
        renderer_ = vtkNew<vtkRenderer> {};
        window_ = vtkNew<vtkGenericOpenGLRenderWindow> {};

        interface->setRenderWindow(window_.Get());
        speed::info("connect widget with renderer");

        auto text = vtkNew<vtkTextActor> {};
        text->SetInput("point-cloud-shop");
        text->SetPosition2(0, 0);
        text->GetTextProperty()->SetFontSize(15);
        text->GetTextProperty()->SetColor(0.233, 0.722, 0.733);

        renderer_->AddActor2D(text);

        window_->SetWindowName("core::Renderer");
        window_->AddRenderer(renderer_);

        vtkNew<PickStyle> style;
        interface->interactor()->SetInteractorStyle(style);

        window_->Render();
    }

    void addCloud(int index, const PointCloud& item) {
        if (cloudActors_.contains(index)) {
            speed::info("cloud model already exits");
            return;
        }

        vtkNew<vtkPoints> points;
        for (const auto point : item.points())
            points->InsertNextPoint(point.x, point.y, point.z);

        vtkNew<vtkPolyData> polyData;
        polyData->SetPoints(points);

        vtkNew<vtkVertexGlyphFilter> vertexFilter;
        vertexFilter->SetInputData(polyData);
        vertexFilter->Update();
        polyData->ShallowCopy(vertexFilter->GetOutput());

        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputData(polyData);

        vtkNew<vtkActor> actor;
        actor->SetMapper(mapper);

        renderer_->AddActor(actor);
        cloudActors_[index] = actor;

        window_->Render();
    }

private:
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> window_;

    std::unordered_map<int, vtkSmartPointer<vtkActor>> cloudActors_;
};

Renderer::Renderer(util::Singleton<Renderer>::token)
    : pimpl_(new Impl) {
    speed::info("load core::Renderer");
}

Renderer::~Renderer() {
    delete pimpl_;
    speed::info("delete core::Renderer");
}

/// Configuration
void Renderer::connectWidget(QVTKOpenGLNativeWidget* interface) {
    pimpl_->connectWidget(interface);
}

/// CRUD
void Renderer::addCloud(int index, const core::cloud::Item& item) {
    pimpl_->addCloud(index, item);
}

void Renderer::removeCloud(int index) { }
void Renderer::removeAllCloud() { }

/// Modify property
void Renderer::modifyVisible(int index, bool flag) { }
void Renderer::modifyPointSize(int index, double size) { }
void Renderer::modifyColor(int index, double r, double g, double b) { }

void Renderer::transformCloud(int index) { }