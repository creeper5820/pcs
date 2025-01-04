#include "3d-view.hh"
#include "core/operator/operator.hh"
#include "core/pointcloud/cloud.hh"

#include <creeper-qt/setting/theme.hh>

#include <QLayout>
#include <QTimer>
#include <QVTKOpenGLNativeWidget.h>

using namespace qt;
using namespace creeper;
using namespace core;

struct ThreeDView::Impl {
    QVTKOpenGLNativeWidget view;
    QTimer renderWatchDog;
};

ThreeDView::ThreeDView(QWidget* parent)
    : RoundedRectangle(parent)
    , pimpl_(new Impl) {
    setBackground(Qt::black);

    auto layout = new QVBoxLayout;
    layout->setSpacing(0);
    layout->setMargin(5);
    layout->addWidget(&pimpl_->view);

    setLayout(layout);

    connect(&pimpl_->renderWatchDog, &QTimer::timeout, [&] { pimpl_->view.setVisible(true); });

    auto& operators = Operators::instance();
    operators.connectWidget(&pimpl_->view);
}

ThreeDView::~ThreeDView() { delete pimpl_; }

void ThreeDView::mousePressEvent(QMouseEvent* event) { }

void ThreeDView::mouseMoveEvent(QMouseEvent* event) { }

void ThreeDView::mouseReleaseEvent(QMouseEvent* event) { }

void ThreeDView::resizeEvent(QResizeEvent* event) {
    pimpl_->view.setVisible(false);
    pimpl_->renderWatchDog.start(100);
    RoundedRectangle::resizeEvent(event);
}