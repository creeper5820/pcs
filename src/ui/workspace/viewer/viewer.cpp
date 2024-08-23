#include "viewer.hpp"
#include "utility/common.hpp"

using namespace workspace;

Viewer::Viewer(QWidget* parent)
    : QWidget(parent)
{
    ui_ = std::make_unique<Ui::WorkspaceViewer>();
    ui_->setupUi(this);

    connect(ui_->Refresh, &QPushButton::clicked, this, &Viewer::refresh_callback);
    connect(ui_->Exit, &QPushButton::clicked, this, &Viewer::exit_callback);

    storage_ = std::make_unique<core::viewer::Storage>();
    storage_->bind_viewer(ui_->vtkWidget);
    storage_->load_cloud("/home/creeper/workspace/sentry/ignore/bag/robomaster_slam.pcd", "123");
    utility::refresh_vtk(ui_->vtkWidget);
}

Viewer::~Viewer()
{
}

void Viewer::refresh_callback()
{
    utility::refresh_vtk(ui_->vtkWidget);
}

void Viewer::exit_callback()
{
    auto answer = QMessageBox::information(NULL, "info", "Sure to exit?",
        QMessageBox::Yes | QMessageBox::No);

    if (answer == QMessageBox::Yes) {
        std::exit(0);
    }
}
