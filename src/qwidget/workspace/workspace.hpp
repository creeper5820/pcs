#pragma once

#include <qmainwindow.h>
#include <qobjectdefs.h>

namespace workspace {

class Workspace final : public QMainWindow {
    Q_OBJECT

public:
    Workspace();
    ~Workspace();
    Workspace(const Workspace&) = delete;
    Workspace& operator=(const Workspace&) = delete;

private Q_SLOTS:

private:
    struct Impl;
    Impl* pimpl_;
};

}