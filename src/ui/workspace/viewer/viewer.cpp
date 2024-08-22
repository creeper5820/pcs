#include "viewer.hpp"
#include "ui_viewer.h"

#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <qchar.h>
#include <qgraphicsscene.h>
#include <qmessagebox.h>
#include <qobjectdefs.h>
#include <qpixmap.h>
#include <qpushbutton.h>
#include <qwidget.h>

#if VTK_MAJOR_VERSION > 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif

namespace workspace::internal {
template <typename... Args>
    requires requires(Args... args) {
        ((std::cout << args), ...);
    }
inline void message(Args... args)
{
    ((std::cout << '[' << args << ']'), ...) << std::endl;
}

inline void bind_vtk(
    std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
    QVTKOpenGLNativeWidget* vtk, const std::string& name)
{
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

inline void refresh_vtk(QVTKOpenGLNativeWidget* vtk)
{
#if VTK_MAJOR_VERSION > 8
    vtk->renderWindow()->Render();
#else
    vtk->update();
#endif
}
}; // namespace workspace::internal

namespace workspace {
struct Viewer::Impl {
public:
    enum class ReturnT : uint8_t {
        SUCCESS,
        NEED_INIT,
    };

    // 注意构造中不要依赖UI，此时还未与 QWidget/QMainWindow 绑定
    Impl()
    {
        ui_ = std::make_unique<Ui::WorkspaceViewer>();
    }

    const std::unique_ptr<Ui::WorkspaceViewer>& ui() { return ui_; }

    void load()
    {
        internal::bind_vtk(viewer_, ui_->vtkWidget, "vtk");
    }

    void refresh()
    {
        internal::refresh_vtk(ui_->vtkWidget);
    }

    template <typename PointT>
        requires requires(PointT) {
            pcl::PointCloud<PointT> {};
        }
    ReturnT register_cloud(pcl::PointCloud<PointT>::Ptr cloud, const std::string& name)
    {
        if (viewer_ == nullptr) {
            internal::message("visualizer need to be initialized");
            return ReturnT::NEED_INIT;
        }

        viewer_->addPointCloud(cloud, name);
        viewer_->resetCamera();

        internal::message("register cloud successfully");
        return ReturnT::SUCCESS;
    }

private:
    std::unique_ptr<Ui::WorkspaceViewer> ui_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
};

Viewer::Viewer(QWidget* parent)
    : QWidget(parent)
{
    pimpl_ = new Impl {};
    pimpl_->ui()->setupUi(this);
    pimpl_->load();

    connect(pimpl_->ui()->Refresh, &QPushButton::clicked, this, &Viewer::refresh_callback);
    connect(pimpl_->ui()->Exit, &QPushButton::clicked, this, &Viewer::exit_callback);

    const auto path = "/home/creeper/workspace/sentry/ignore/bag/robomaster_slam2.pcd";

    static auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile(path, *cloud) != -1) {
        internal::message("cloud size", cloud->size());
        pimpl_->register_cloud<pcl::PointXYZ>(cloud, "cloud");
        pimpl_->refresh();
    } else {
        internal::message("load cloud failed", path);
        std::exit(-1);
    }
}

Viewer::~Viewer()
{
    delete pimpl_;
}

void Viewer::refresh_callback()
{
    pimpl_->refresh();
    internal::message("refreshed");
}

void Viewer::exit_callback()
{
    auto answer = QMessageBox::information(NULL, "Exits", "Sure to exit?",
        QMessageBox::Yes | QMessageBox::No);

    if (answer == QMessageBox::Yes) {
        std::exit(0);
    } else {
        internal::message("cancel exit");
    }

    auto s = QString {};
}

} // namespace workspace