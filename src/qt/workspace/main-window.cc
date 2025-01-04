#include <qfileinfo.h>
#include <qmenubar.h>
#include <qpointer.h>
#include <qtoolbar.h>

#include <creeper-qt/widget/push-button.hh>
#include <spdlog/spdlog.h>

#include "3d-view.hh"
#include "main-window.hh"
#include "top-area.hh"
#include "workbench/workbench.hh"

using namespace creeper;
using namespace qt;

struct Workspace::Impl {
public:
    QPointer<QWidget> widget() {
        auto view = new ThreeDView;
        auto workBench = new WorkBench;
        auto topArea = new TopArea;

        connect(workBench, &WorkBench::openFileFromDevice, [topArea](const QString& path) {
            auto name = QFileInfo(path).fileName();
            topArea->setFileName(name);
        });

        auto horizon = new QHBoxLayout;
        horizon->setMargin(5);
        horizon->setSpacing(5);
        horizon->addWidget(view);
        horizon->addWidget(workBench);

        auto vertical = new QVBoxLayout;
        vertical->setMargin(0);
        vertical->setSpacing(0);
        vertical->addWidget(topArea);
        vertical->addLayout(horizon);

        auto mainWidget = new QWidget;
        mainWidget->setLayout(vertical);

        return mainWidget;
    }
};

Workspace::Workspace()
    : pimpl_(new Impl) {
    setCentralWidget(pimpl_->widget());
    setWindowTitle("pcs");
}

Workspace::~Workspace() { delete pimpl_; }