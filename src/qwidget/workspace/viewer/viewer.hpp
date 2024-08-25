#pragma once

#include "core/viewer/cloud.hpp"
#include "utility/common.hpp"

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
        page_ = new Ui::WorkspaceViewer();
        page_->setupUi(this);

        page_->frame->setStyleSheet(
            utility::style(":qss/normal/viewer.qss"));

        core::viewer::storage.bind_viewer(page_->vtkWidget);
    }

    ~Viewer() { }

private:
    Ui::WorkspaceViewer* page_;
};

} // namespace workspace