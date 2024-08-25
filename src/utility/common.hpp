#pragma once

#include <QVTKOpenGLNativeWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <qfileinfo.h>

#if VTK_MAJOR_VERSION > 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif

namespace utility {
template <typename... Args>
    requires requires(Args... args) { ((std::cout << args), ...); }
inline void message(Args... args) {
    ((std::cout << '[' << args << ']'), ...) << std::endl;
}

inline void bind_vtk(std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
    QVTKOpenGLNativeWidget* vtk, const std::string& name) {
    using namespace pcl::visualization;

#if VTK_MAJOR_VERSION > 8
    auto render = vtkSmartPointer<vtkRenderer>::New();
    auto window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    window->AddRenderer(render);
    viewer = std::make_shared<PCLVisualizer>(render, window, "viewer", false);
    vtk->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vtk->interactor(), vtk->renderWindow());
#else
    viewer.reset(new PCLVisualizer(name, false));
    vtk->SetRenderWindow(viewer_->getRenderWindow());
    viewer->setupInteractor(vtk->GetInteractor(), vtk->GetRenderWindow());
#endif
}

inline void refresh_vtk(QVTKOpenGLNativeWidget* vtk) {
#if VTK_MAJOR_VERSION > 8
    vtk->renderWindow()->Render();
#else
    vtk->update();
#endif
}

inline const QString style(const QString& url) {
    QFile style { url };
    style.open(QFile::ReadOnly | QFile::Text);
    return style.readAll();
}
};