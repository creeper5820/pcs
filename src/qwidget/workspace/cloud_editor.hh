#pragma once
#include "cloud_list_item.hh"
#include "core/cloud/cloud.hh"

#include <spdlog/spdlog.h>

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

namespace workspace {
namespace speed = spdlog;

class CloudEditor : public QWidget, Ui::CloudEditor {
    Q_OBJECT
public:
    CloudEditor(QWidget* parent = nullptr)
        : QWidget(parent) {
        setupUi(this);

        // bind icon with button
        clearButton->setIcon(QPixmap(":pic/clear.svg"));
        loadButton->setIcon(QPixmap(":pic/file.svg"));
        refreshButton->setIcon(QPixmap(":pic/reset.svg"));

        // custom context menu
        cloudList->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(cloudList, &QWidget::customContextMenuRequested,
            this, &CloudEditor::showListMenu);

        // connect
        connect(refreshButton, &QPushButton::clicked,
            this, &CloudEditor::refresh);
        connect(loadButton, &QPushButton::clicked,
            this, &CloudEditor::loadCloud);
        connect(clearButton, &QPushButton::clicked,
            this, &CloudEditor::removeAllCloud);
        connect(cloudList, &QListWidget::itemClicked,
            this, &CloudEditor::updateEditorMessage);
        connect(nameEditor, &QLineEdit::editingFinished,
            this, &CloudEditor::applyCloudName);
        connect(frameEditor, &QLineEdit::editingFinished,
            this, &CloudEditor::applyCloudFrame);
        connect(colorEditor, &QLineEdit::editingFinished,
            this, &CloudEditor::applyCloudColor);
        connect(pointSizeEditor, &QLineEdit::editingFinished,
            this, &CloudEditor::applyPointSize);
    }
private slots:
    /// Operators
    void loadCloud() {
        const auto path = QFileDialog::getOpenFileName(this,
            "Open File", "", "PCD Files (*.pcd)");
        if (path.isEmpty() || paths_.contains(path))
            return;

        const auto pathStd = path.toStdString();
        speed::info("load cloud form {}", pathStd);

        auto index = cloudManager_.loadCloud(pathStd);
        cloudManager_.modifyPointSize(index, 1);

        auto item = new CloudListItem(cloudList);
        item->setCloudIndex(index);
        item->setPath(path);
        item->setLabel(getNameFromPath(path));
    }

    void removeSelectedCloud() {
        auto* item = dynamic_cast<CloudListItem*>(
            cloudList->currentItem());
        auto cloudIndex = item->cloudIndex();
        cloudManager_.removeCloud(cloudIndex);
        delete item;
    }

    void refresh() {
        cloudManager_.refresh();
    }

    void removeAllCloud() {
        cloudList->clear();
        cloudManager_.removeAllCloud();
    }

    /// List Widget
    void showListMenu() {
        auto menu = QMenu {};
        menu.addAction("delete", this,
            &CloudEditor::removeSelectedCloud);
        menu.exec(QCursor::pos());
    }

    /// Editor
    void updateEditorMessage() {
        auto item = currentListItem();

        if (item == nullptr) {
            nameEditor->setText("");
            colorEditor->setText("");
            frameEditor->setText("");
            return;
        }

        nameEditor->setText(item->label());
        frameEditor->setText(item->frame());
        pointSizeEditor->setText(QString::number(item->pointSize()));
        colorEditor->setText(QString::asprintf(
            "%.3f, %.3f, %.3f", item->r(), item->g(), item->b()));
    }

    void applyCloudName() {
        auto input = nameEditor->text();
        auto target = input.contains(".") ? input : input + ".pcd";
        currentListItem()->setLabel(target);
    }

    void applyCloudColor() {
        static const auto regexRGB = std::regex(
            "\\s*(0(?:\\.0*)?|1(?:\\.0*)?|"
            "\\d*\\.?\\d*|"
            "0?\\.\\d*[1-9])\\s*,"
            "\\s*(0(?:\\.0*)?|1(?:\\.0*)?|"
            "\\d*\\.?\\d*|"
            "0?\\.\\d*[1-9])\\s*,"
            "\\s*(0(?:\\.0*)?|1(?:\\.0*)?|"
            "\\d*\\.?\\d*|"
            "0?\\.\\d*[1-9])\\s*");
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
            r = std::stoul(match[1], 0, 16) / 255.0;
            g = std::stoul(match[2], 0, 16) / 255.0;
            b = std::stoul(match[3], 0, 16) / 255.0;
        } else {
            speed::info("invalid color: {}", text);
            return;
        }
        currentListItem()->setColor(r, g, b);
    }

    void applyCloudFrame() {
        currentListItem()->setFrame(frameEditor->text());
    }

    void applyPointSize() {
        auto text = pointSizeEditor->text();
        auto size = text.toInt();
        if (size > 0)
            currentListItem()->setPointSize(size);
    }

private:
    QStringList paths_;
    QStringList names_;

    core::Cloud& cloudManager_ = core::Cloud::instance();

    inline CloudListItem* currentListItem() {
        return dynamic_cast<CloudListItem*>(
            cloudList->currentItem());
    }

    static inline QString getNameFromPath(const QString& path) {
        return path.right(path.size() - path.lastIndexOf("/") - 1);
    }
};

} // namespace workspace