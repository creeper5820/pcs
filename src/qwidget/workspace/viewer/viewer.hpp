#pragma once

#include "core/viewer/cloud.hpp"

#include <qmessagebox.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <ui_viewer.h>

namespace workspace {

class Viewer : public QWidget {
    Q_OBJECT
public:
    Viewer(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        ui_ = new Ui::WorkspaceViewer();
        ui_->setupUi(this);
        core::viewer::storage.bind_viewer(ui_->vtkWidget);
    }

    ~Viewer() { }

private:
    Ui::WorkspaceViewer* ui_;
};

} // namespace workspace