#pragma once

#include <creeper-qt/widget/basic-shape.hh>

#include "utility/common.hh"

namespace qt {

class WorkBench : public creeper::RoundedRectangle {
    WIDGET_PIMPL_DEFINTION(WorkBench);
    Q_OBJECT

signals:
    void openFileFromDevice(const QString& path);
};

}