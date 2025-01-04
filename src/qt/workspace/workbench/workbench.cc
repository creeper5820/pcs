#include "workbench.hh"
#include "file-explorer.hh"

#include <creeper-qt/module/round-icon-button.hh>
#include <creeper-qt/setting/theme.hh>
#include <creeper-qt/widget/push-button.hh>

#include <spdlog/spdlog.h>

#include <qboxlayout.h>
#include <qfiledialog.h>

using namespace qt;
using namespace creeper;

struct WorkBench::Impl {
    QuickAutoTheme<FileExplorer> fileExplorer { [](RoundedRectangle& widget) {
        widget.setBackground(Theme::color("primary050"));
    } };
};

WorkBench::WorkBench(QWidget* parent)
    : pimpl_(new Impl) {
    setBackground(Theme::color("background"));
    setMinimumWidth(300);
    setMaximumWidth(500);

    pimpl_->fileExplorer.setFixedHeight(200);
    pimpl_->fileExplorer.setBackground(Theme::color("primary050"));

    auto vertical = new QVBoxLayout;
    vertical->setAlignment(Qt::AlignTop);
    vertical->setSpacing(0);
    vertical->setMargin(0);
    vertical->addWidget(&pimpl_->fileExplorer);

    setLayout(vertical);

    connect(&pimpl_->fileExplorer, &FileExplorer::openFileFromDevice, this,
        &WorkBench::openFileFromDevice);
}

WorkBench::~WorkBench() { delete pimpl_; }