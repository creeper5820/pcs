#include "file-explorer.hh"
#include "core/pointcloud/cloud.hh"

#include <creeper-qt/module/round-icon-button.hh>
#include <qfiledialog.h>
#include <qtimer.h>

#include <spdlog/spdlog.h>

using namespace qt;
using namespace creeper;

struct FileExplorer::Impl {
    RoundIconButton loadFile;
    RoundIconButton clearFile;

    QTimer readFileAsyncChecker;
    std::unique_ptr<CloudPackage> asyncPackage;

    std::vector<std::unique_ptr<CloudPackage>> packages;
};

FileExplorer::FileExplorer(QWidget* parent)
    : pimpl_(new Impl) {
    pimpl_->loadFile.setRadius(20);
    pimpl_->loadFile.setIcon(QIcon(":/theme/icon/normal/search.png"));

    pimpl_->clearFile.setRadius(20);
    pimpl_->clearFile.setIcon(QIcon(":/theme/icon/normal/menu.png"));

    auto horizon = new QHBoxLayout;
    horizon->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    horizon->addWidget(&pimpl_->loadFile);
    horizon->addWidget(&pimpl_->clearFile);

    setLayout(horizon);

    /// @todo replace this file dialog, it's not a good idea to use it
    connect(&pimpl_->loadFile, &QPushButton::clicked, [this] {
        const auto filter = "PCD Files (*.pcd)";
        const auto directory = "";
        const auto caption = "Open PCD File";
        const auto path = QFileDialog::getOpenFileName(this, caption, directory, filter);
        if (path.isEmpty()) return;

        auto& manager = CloudManager::instance();
        pimpl_->asyncPackage = manager.makePackage(path);

        using namespace std::chrono_literals;
        pimpl_->readFileAsyncChecker.start(500ms);

        spdlog::info("load cloud form {}", path.toStdString());
        emit openFileFromDevice(path);
    });

    connect(&pimpl_->clearFile, &QPushButton::clicked, [this] {
        pimpl_->packages.clear(); //
        spdlog::info("clear all pointcloud");
    });

    connect(&pimpl_->readFileAsyncChecker, &QTimer::timeout, [this] {
        if (!pimpl_->asyncPackage) return;
        if (pimpl_->asyncPackage->tryToLoadRenderer()) {
            pimpl_->packages.push_back(std::move(pimpl_->asyncPackage));
            pimpl_->asyncPackage.reset();
            pimpl_->readFileAsyncChecker.stop();
            spdlog::info("load cloud done");
        }
    });
}

FileExplorer::~FileExplorer() { delete pimpl_; }
