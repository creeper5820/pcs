#pragma once

#include <Eigen/Dense>
#include <qlistwidget.h>

#include "core/cloud/cloud.hh"

static auto& cloudManager = core::Cloud::instance();

namespace workspace {
class CloudListItem : public QListWidgetItem {
public:
    CloudListItem(QListWidget* list = nullptr)
        : QListWidgetItem(list) {
    }

    void setPath(const QString& path) {
        path_ = path;
    }

    void setLabel(const QString& label) {
        setText(label);
    }

    void setCloudIndex(int cloudIndex) {
        cloudIndex_ = cloudIndex;
    }

    void setFrame(const QString& frame) {
        frame_ = frame;
    }

    void setPointSize(double pointSize) {
        cloudManager.modifyPointSize(cloudIndex_, pointSize);
        pointSize_ = pointSize;
    }

    void setVisible(bool visible) {
        cloudManager.modifyVisible(cloudIndex_, visible);
        visible_ = visible;
    }

    void setColor(double r, double g, double b) {
        cloudManager.modifyColor(cloudIndex_, r, g, b);
        r_ = r;
        g_ = g;
        b_ = b;
    }

    QString path() const { return path_; }
    QString label() const { return text(); }
    QString frame() const { return frame_; }
    double r() const { return r_; }
    double g() const { return g_; }
    double b() const { return b_; }
    int cloudIndex() const { return cloudIndex_; }
    double pointSize() const { return pointSize_; }
    bool visible() const { return visible_; }

private:
    QString path_ = "unknown";
    QString frame_ = "unknown";
    int cloudIndex_ = -1;

    double r_ = 1;
    double g_ = 1;
    double b_ = 1;

    double pointSize_ = 1;
    bool visible_ = true;
};

}