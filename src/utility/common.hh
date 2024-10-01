#pragma once

#include <QVTKOpenGLNativeWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <qfileinfo.h>

#if VTK_MAJOR_VERSION > 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif

namespace util {
template <typename... Args>
    requires requires(Args... args) { ((std::cout << args), ...); }
inline void message(Args... args) {
    ((std::cout << '[' << args << ']'), ...) << std::endl;
}

inline const QString style(const QString& url) {
    QFile style { url };
    style.open(QFile::ReadOnly | QFile::Text);
    return style.readAll();
}
};