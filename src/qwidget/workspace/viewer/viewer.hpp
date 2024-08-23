#pragma once
#include "core/viewer/cloud.hpp"
#include "ui_viewer.h"

#include <qmessagebox.h>
#include <qpushbutton.h>
#include <qwidget.h>

#include <memory>

namespace workspace {

class Viewer : public QWidget {
    Q_OBJECT
public:
    Viewer(QWidget* parent = nullptr);
    ~Viewer();

private Q_SLOTS:
    void refresh_callback();
    void exit_callback();

private:
    std::unique_ptr<Ui::WorkspaceViewer> ui_;
    std::unique_ptr<core::viewer::Storage> storage_;
};

} // namespace workspace