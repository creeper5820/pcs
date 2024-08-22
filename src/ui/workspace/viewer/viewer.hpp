#pragma once

#include <qwidget.h>

namespace workspace {

class Viewer : public QWidget {
    Q_OBJECT
public:
    Viewer(QWidget* parent = nullptr);
    ~Viewer();
    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

private Q_SLOTS:
    void refresh_callback();
    void exit_callback();

private:
    struct Impl;
    Impl* pimpl_;
};

} // namespace workspace