#pragma once
#include "core/pointcloud/cloud.hh"
#include "widget/workspace/list-item.hh"

#include <qchar.h>
#include <qcursor.h>
#include <qfiledialog.h>
#include <qglobal.h>
#include <qlabel.h>
#include <qlist.h>
#include <qlistwidget.h>
#include <qmenu.h>
#include <qpixmap.h>
#include <qwidget.h>
#include <ui_cloud_editor.h>

#include <regex>
#include <spdlog/spdlog.h>

namespace workspace {
namespace speed = spdlog;

class CloudEditor : public QWidget, Ui::CloudEditor {
    Q_OBJECT
public:
    explicit CloudEditor(QWidget* parent = nullptr)
        : QWidget(parent) {
        setupUi(this);

        // bind icon with button
        clearButton->setIcon(QPixmap(":pic/clear.svg"));
        loadButton->setIcon(QPixmap(":pic/file.svg"));
        refreshButton->setIcon(QPixmap(":pic/reset.svg"));

        // custom context menu
        cloudList->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(cloudList, &QWidget::customContextMenuRequested,
            this, &CloudEditor::onShowListMenu);

        // connect
        connect(refreshButton, &QPushButton::clicked,
            this, &CloudEditor::onRefresh);
        connect(loadButton, &QPushButton::clicked,
            this, &CloudEditor::onLoadCloud);
        connect(clearButton, &QPushButton::clicked,
            this, &CloudEditor::onClear);

        connect(cloudList, &QListWidget::itemClicked,
            this, &CloudEditor::onItemClicked);

        connect(nameEditor, &QLineEdit::editingFinished,
            this, &CloudEditor::onNameChanged);
        connect(frameEditor, &QLineEdit::editingFinished,
            this, &CloudEditor::onFrameChanged);
        connect(colorEditor, &QLineEdit::editingFinished,
            this, &CloudEditor::onColorChanged);
        connect(pointSizeEditor, &QLineEdit::editingFinished,
            this, &CloudEditor::onSizeChanged);

        connect(visibleCheck, &QCheckBox::stateChanged,
            this, &CloudEditor::onVisbleCheckClicked);
    }

private slots:
    /// Operators
    void onLoadCloud() {
        const auto path = QFileDialog::getOpenFileName(this,
            "Open File", "", "PCD Files (*.pcd)");
        if (path.isEmpty() || paths_.contains(path))
            return;

        const auto pathStd = path.toStdString();
        speed::info("load cloud form {}", pathStd);

        auto package = cloudManager_.makePackage(pathStd);

        auto* listItem = new CloudListItem(cloudList);
        listItem->bindPackage(std::move(package));
        listItem->setLabel(getNameFromPath(path));
    }

    void onRemoveSelected() {
        auto item = currentListItem();
        if (item == nullptr)
            return;
        auto removed = cloudList->takeItem(
            cloudList->row(item));
        // 显式地删除元素，各版本Qt行为不同
        delete removed;
    }

    void onRefresh() {
        cloudManager_.refresh();
    }

    void onClear() {
        cloudList->clear();
    }

    /// List Widget
    void onShowListMenu() const {
        auto menu = QMenu {};
        menu.addAction("delete", this,
            &CloudEditor::onRemoveSelected);
        menu.exec(QCursor::pos());
    }

    /// Editor
    void onItemClicked() {
        auto item = currentListItem();

        if (item == nullptr)
            return;

        auto [r, g, b] = item->color();
        auto sizeText = QString::number(item->pointSize());
        auto colorText = QString::asprintf("%.3f, %.3f, %.3f", r, g, b);

        nameEditor->setText(item->label());
        frameEditor->setText(item->frame());
        pointSizeEditor->setText(sizeText);
        colorEditor->setText(colorText);
        visibleCheck->setChecked(item->visible());
    }

    void onNameChanged() {
        auto input = nameEditor->text();
        auto target = input.contains(".") ? input : input + ".pcd";
        currentListItem()->setLabel(target);
    }

    void onColorChanged() {
        static const auto regexRGB = std::regex(
            "\\s*(0(?:\\.0*)?|1(?:\\.0*)?|\\d*\\.?\\d*|0?\\.\\d*[1-9])\\s*,"
            "\\s*(0(?:\\.0*)?|1(?:\\.0*)?|\\d*\\.?\\d*|0?\\.\\d*[1-9])\\s*,"
            "\\s*(0(?:\\.0*)?|1(?:\\.0*)?|\\d*\\.?\\d*|0?\\.\\d*[1-9])\\s*");
        static const auto regexHEX = std::regex(
            "^\\s*#([0-9|a-f|A-F]{2})([0-9|a-f|A-F]{2})([0-9|a-f|A-F]{2})\\s*$");

        double r, g, b;
        const auto text = colorEditor->text().toStdString();

        auto match = std::smatch();
        if (std::regex_match(text, match, regexRGB)) {
            r = std::stod(match[1]);
            g = std::stod(match[2]);
            b = std::stod(match[3]);
        } else if (std::regex_match(text, match, regexHEX)) {
            // TODO: handle hex color
        } else {
            speed::warn("invalid color: {}", text);
            return;
        }
        currentListItem()->setColor(r, g, b);
    }

    void onFrameChanged() {
        currentListItem()->setFrame(frameEditor->text());
    }

    void onSizeChanged() {
        const auto text = pointSizeEditor->text();
        const auto size = text.toFloat();
        currentListItem()->setPointSize(size);
    }

    void onVisbleCheckClicked() {
        bool visible = visibleCheck->checkState();
        currentListItem()->setVisible(visible);
    }

private:
    QStringList paths_;
    QStringList names_;

    CloudManager& cloudManager_ = CloudManager::instance();

    inline CloudListItem* currentListItem() {
        return dynamic_cast<CloudListItem*>(
            cloudList->currentItem());
    }

    static inline QString getNameFromPath(const QString& path) {
        return path.right(path.size() - path.lastIndexOf("/") - 1);
    }
};

} // namespace workspace