#pragma once
#include "core/viewer/cloud.hpp"
#include "utility/common.hpp"

#include <qchar.h>
#include <qcursor.h>
#include <qfiledialog.h>
#include <qlabel.h>
#include <qlist.h>
#include <qlistwidget.h>
#include <qmenu.h>
#include <qpixmap.h>
#include <qwidget.h>
#include <ui_cloud_item.h>
#include <ui_tool_bar.h>

namespace workspace {
class ToolBar : public QWidget {
    Q_OBJECT
public:
    ToolBar(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        page_.setupUi(this);

        page_.button_clear->setIcon(QPixmap(":pic/clear.svg"));
        page_.button_load->setIcon(QPixmap(":pic/file.svg"));
        page_.button_refresh->setIcon(QPixmap(":pic/reset.svg"));

        connect(page_.button_refresh, &QPushButton::clicked, this, &ToolBar::refresh);
        connect(page_.button_load, &QPushButton::clicked, this, &ToolBar::load);
        connect(page_.button_clear, &QPushButton::clicked, this, &ToolBar::clear);

        page_.list_cloud->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(page_.list_cloud, &QWidget::customContextMenuRequested, [this]() {
            auto menu = QMenu {};
            menu.addAction("delete", this, &ToolBar::remove_current_cloud);
            menu.exec(QCursor::pos());
        });
    }
private slots:
    void remove_current_cloud()
    {
        const auto index = page_.list_cloud->currentRow();
        storage_.remove_cloud(cloud_path_[index].toStdString());
        remove_from_list(index);
    }
    void refresh()
    {
        storage_.refresh_viewer();
    }
    void load()
    {
        const auto path = QFileDialog::getOpenFileName(
            this, tr("Open File"), "/path/to/file/", tr("PCD Files (*.pcd)"))
                              .toStdString();
        const auto name = path.substr(path.find_last_of("/") + 1);

        if (!path.empty()) {
            if (!cloud_path_.contains(QString::fromStdString(path))
                && storage_.load_cloud(path, path))
                append_to_list(QString::fromStdString(path));
        }
    }
    void clear()
    {
        storage_.clear_cloud();
        clear_list();
    }

private:
    core::viewer::Storage& storage_ { core::viewer::storage };

    Ui::ToolBar page_;
    QStringList cloud_path_;
    QStringList cloud_name_;

    static inline auto get_name_form_path(const QString& path)
    {
        return path.right(path.size() - path.lastIndexOf("/") - 1);
    }

    inline void refresh_list()
    {
        page_.list_cloud->clear();
        page_.list_cloud->addItems(cloud_name_);
    }
    inline void append_to_list(const QString& path)
    {
        cloud_path_.append(path);
        cloud_name_.append(get_name_form_path(path));
        refresh_list();
    }
    inline void remove_from_list(int index)
    {
        cloud_path_.removeAt(index);
        cloud_name_.removeAt(index);
        refresh_list();
    }
    inline void remove_from_list(const QString& path)
    {
        cloud_path_.removeOne(path);
        cloud_name_.removeOne(get_name_form_path(path));
        refresh_list();
    }
    inline void clear_list()
    {
        cloud_path_.clear();
        cloud_name_.clear();
        refresh_list();
    }
};

}