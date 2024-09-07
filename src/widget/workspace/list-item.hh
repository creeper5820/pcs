#pragma once

#include "core/pointcloud/cloud.hh"
#include "core/renderer/renderer.hh"

#include <qlistwidget.h>

static auto& cloudManager = CloudManager::instance();

namespace workspace {
class CloudListItem : public QListWidgetItem {
public:
    explicit CloudListItem(QListWidget* list = nullptr)
        : QListWidgetItem(list) {
    }

    void bindPackage(std::unique_ptr<CloudPackage> package) {
        package_ = std::move(package);
        auto [r, g, b] = color_;
        package_->object->setPointSize(pointSize_);
        package_->object->setVisible(visible_);
        package_->object->setColor(r, g, b);
        renderer_.render();
    }

    void setPath(const QString& path) {
        path_ = path;
    }

    void setLabel(const QString& label) {
        setText(label);
    }

    void setFrame(const QString& frame) {
        frame_ = frame;
    }

    void setPointSize(double pointSize) {
        package_->object->setPointSize(pointSize);
        renderer_.render();
        pointSize_ = pointSize;
    }

    void setVisible(bool visible) {
        package_->object->setVisible(visible);
        renderer_.render();
        visible_ = visible;
    }

    void setPickable(bool pickable) {
        package_->object->setPickable(pickable);
        renderer_.render();
        pickable_ = pickable;
    }

    void setColor(double r, double g, double b) {
        package_->object->setColor(r, g, b);
        renderer_.render();
        color_ = { r, g, b };
    }

    QString path() const { return path_; }
    QString label() const { return text(); }
    QString frame() const { return frame_; }
    RenderColor color() const { return color_; }
    double pointSize() const { return pointSize_; }
    bool visible() const { return visible_; }
    bool pickable() const { return pickable_; }

private:
    std::unique_ptr<CloudPackage> package_;
    RenderColor color_ { 1, 1, 1 };
    double pointSize_ { 2.0 };
    bool visible_ { true };
    bool pickable_ { true };

    Renderer& renderer_ = Renderer::instance();

    QString path_ = "unknown";
    QString frame_ = "unknown";
};

}