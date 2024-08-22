#include "viewer.hpp"

#include <qgraphicsscene.h>
#include <qobjectdefs.h>
#include <qpixmap.h>
#include <qwidget.h>

#include <cstdio>
#include <memory>

#include "ui_viewer.h"

#if VTK_MAJOR_VERSION > 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace workspace;

struct Viewer::Impl {
 public:
  Impl() {
    // 注意构造中不要依赖UI，此时还未与 QWidget/QMainWindow 绑定
    ui_ = std::make_unique<Ui::WorkspaceViewer>();
    scene_ = std::make_unique<QGraphicsScene>();
  }

  ~Impl() {}

  inline void show_picture() {
    ui_->graphicsView->setScene(scene_.get());
    ui_->graphicsView->show();
  }

  inline void close_picture() { ui_->graphicsView->close(); }

  void init_vtk_viewer() {
    using namespace pcl::visualization;
    auto& vtk = ui_->vtkWidget;
    auto name = std::string("viewer");

#if VTK_MAJOR_VERSION > 8
    auto render = vtkSmartPointer<vtkRenderer>::New();
    auto window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    window->AddRenderer(render);
    viewer_.reset(
        new pcl::visualization::PCLVisualizer(render, window, "viewer", false));
    vtk->setRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(vtk->interactor(), vtk->renderWindow());
#else
    viewer_.reset(new PCLVisualizer(name, false));
    vtk->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(vtk->GetInteractor(), vtk->GetRenderWindow());
#endif
  }

  const std::unique_ptr<Ui::WorkspaceViewer>& ui() { return ui_; }

 private:
  std::unique_ptr<Ui::WorkspaceViewer> ui_;
  std::unique_ptr<QGraphicsScene> scene_;

  pcl::visualization::PCLVisualizer::Ptr viewer_;
};

Viewer::Viewer(QWidget* parent) : QWidget(parent) {
  pimpl_ = new Impl{};
  pimpl_->ui()->setupUi(this);
  pimpl_->init_vtk_viewer();

  connect(pimpl_->ui()->hello_world, &QPushButton::clicked, this,
          &Viewer::button_hello_world_callback);
}

Viewer::~Viewer() { delete pimpl_; }

void Viewer::button_hello_world_callback() {}