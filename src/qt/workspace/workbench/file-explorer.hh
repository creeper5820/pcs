#pragma once

#include "utility/common.hh"
#include <creeper-qt/widget/basic-shape.hh>

namespace qt {
class FileExplorer : public creeper::RoundedRectangle {
    WIDGET_PIMPL_DEFINTION(FileExplorer);
    Q_OBJECT

signals:
    void openFileFromDevice(const QString& path);
};
}