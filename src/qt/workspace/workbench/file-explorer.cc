#include "file-explorer.hh"

#include "core/pointcloud/cloud.hh"

#include <creeper-qt/module/round-icon-button.hh>
#include <qfiledialog.h>
#include <spdlog/spdlog.h>

using namespace qt;
using namespace creeper;

struct FileExplorer::Impl {
    RoundIconButton loadFile;
    RoundIconButton saveFile;
    std::vector<std::unique_ptr<CloudPackage>> packages;
};

FileExplorer::FileExplorer(QWidget* parent)
    : pimpl_(new Impl) {
    pimpl_->loadFile.setRadius(20);
    pimpl_->loadFile.setIcon(QIcon(":/theme/icon/normal/search.png"));

    pimpl_->saveFile.setRadius(20);
    pimpl_->saveFile.setIcon(QIcon(":/theme/icon/normal/menu.png"));

    auto horizon = new QHBoxLayout;
    horizon->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    horizon->addWidget(&pimpl_->loadFile);
    horizon->addWidget(&pimpl_->saveFile);

    setLayout(horizon);

    /// @todo replace this file dialog, it's not a good idea to use it
    connect(&pimpl_->loadFile, &QPushButton::clicked, [this] {
        const auto filter = "PCD Files (*.pcd)";
        const auto directory = "";
        const auto caption = "Open PCD File";
        const auto path = QFileDialog::getOpenFileName(this, caption, directory, filter);
        if (path.isEmpty()) return;

        auto& manager = CloudManager::instance();
        pimpl_->packages.push_back(std::move(manager.makePackage(path)));

        spdlog::info("load cloud form {}", path.toStdString());
        emit openFileFromDevice(path);
    });
}

FileExplorer::~FileExplorer() { delete pimpl_; }
