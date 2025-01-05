#include "main-window.hh"
#include "3d-view/3d-view.hh"
#include "top-area/top-area.hh"
#include "workbench/workbench.hh"

#include <creeper-qt/widget/push-button.hh>

#include <qapplication.h>
#include <qfileinfo.h>
#include <qmenubar.h>
#include <qpointer.h>
#include <qscreen.h>
#include <qtoolbar.h>

#include <spdlog/spdlog.h>

using namespace creeper;
using namespace qt;

struct Workspace::Impl {
public:
    QPointer<QWidget> widget() {
        auto view = new ThreeDView;
        auto workBench = new WorkBench;
        auto topArea = new TopArea;

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

        connect(workBench, &WorkBench::openFileFromDevice, [topArea](const QString& path) {
            auto name = QFileInfo(path).fileName();
            topArea->setFileName(name);
        });

        return mainWidget;
    }
};

Workspace::Workspace()
    : pimpl_(new Impl) {
    setCentralWidget(pimpl_->widget());
    setWindowTitle("pcs");

    auto screenSize = QGuiApplication::primaryScreen()->size();
    QMainWindow::setFixedSize(screenSize * 0.8);
}

Workspace::~Workspace() { delete pimpl_; }