#pragma once
#include "core/viewer/cloud.hpp"

#include <qchar.h>
#include <qcursor.h>
#include <qfiledialog.h>
#include <qlabel.h>
#include <qlist.h>
#include <qlistwidget.h>
#include <qmenu.h>
#include <qpixmap.h>
#include <qwidget.h>
#include <ui_tool_bar.h>

namespace workspace {
class ToolBar : public QWidget {
    Q_OBJECT
public:
    ToolBar(QWidget* parent = nullptr)
        : QWidget(parent) {
        page_.setupUi(this);

        page_.list_cloud->setContextMenuPolicy(Qt::CustomContextMenu);

        page_.button_clear->setIcon(QPixmap(":pic/clear.svg"));
        page_.button_load->setIcon(QPixmap(":pic/file.svg"));
        page_.button_refresh->setIcon(QPixmap(":pic/reset.svg"));

        connect(page_.button_refresh, &QPushButton::clicked,
            this, &ToolBar::refresh);
        connect(page_.button_load, &QPushButton::clicked,
            this, &ToolBar::load_cloud);
        connect(page_.button_clear, &QPushButton::clicked,
            this, &ToolBar::clear_cloud);
        connect(page_.list_cloud, &QListWidget::itemClicked,
            this, &ToolBar::update_cloud_editor);
        connect(page_.cloud_name, &QLineEdit::editingFinished,
            this, &ToolBar::update_cloud_name);
        connect(page_.list_cloud, &QWidget::customContextMenuRequested,
            this, &ToolBar::display_menu_of_list);
    }
private slots:
    void remove_current_cloud() {
        const auto index = get_cloud_index();
        if (index == -1)
            return;
        storage_.remove_cloud(cloud_path_[index].toStdString());
        remove_from_list(index);
    }

    void display_menu_of_list() {
        if (get_cloud_index() == -1)
            return;
        auto menu = QMenu {};
        menu.addAction("delete", this,
            &ToolBar::remove_current_cloud);
        menu.exec(QCursor::pos());
    }

    void update_cloud_editor() {
        const auto index = get_cloud_index();
        const auto path = cloud_path_[index];
        auto& cloud = storage_[path.toStdString()]->get();

        page_.cloud_name->setText(cloud_name_[index]);
        page_.cloud_color->setText(QString::asprintf("%d, %d, %d",
            cloud.color().r, cloud.color().g, cloud.color().b));
        page_.cloud_frame->setText(QString::fromStdString(cloud.frame()));
    }

    void update_cloud_name() {
        auto index = get_cloud_index();
        if (index == -1)
            return;

        auto input = page_.cloud_name->text();
        auto target = input.contains(".")
            ? input
            : input + ".pcd";

        cloud_name_[index] = target;
        refresh_list();
    }

    void update_cloud_color() {
        auto index = get_cloud_index();
        if (index == -1)
            return;

        // TODO: 正则表达式筛选合法颜色值
    }

    void refresh() {
        storage_.refresh_viewer();
    }

    void load_cloud() {
        const auto q_path = QFileDialog::getOpenFileName(
            this, "Open File", "", "PCD Files (*.pcd)");
        const auto path = q_path.toStdString();
        const auto name = path.substr(path.find_last_of("/") + 1);

        if (!path.empty() && !cloud_path_.contains(QString::fromStdString(path))
            && storage_.load_cloud(path))
            append_to_list(QString::fromStdString(path));
    }

    void clear_cloud() {
        storage_.clear_cloud();
        clear_list();
    }

private:
    core::viewer::Storage& storage_ { core::viewer::storage };

    Ui::ToolBar page_;
    QStringList cloud_path_;
    QStringList cloud_name_;

    static inline auto get_name_form_path(const QString& path) {
        return path.right(path.size() - path.lastIndexOf("/") - 1);
    }

    inline void refresh_list() {
        page_.list_cloud->clear();
        page_.list_cloud->addItems(cloud_name_);
    }
    inline void append_to_list(const QString& path) {
        cloud_path_.append(path);
        cloud_name_.append(get_name_form_path(path));
        refresh_list();
    }
    inline void remove_from_list(int index) {
        cloud_path_.removeAt(index);
        cloud_name_.removeAt(index);
        refresh_list();
    }
    inline void remove_from_list(const QString& path) {
        cloud_path_.removeOne(path);
        cloud_name_.removeOne(get_name_form_path(path));
        refresh_list();
    }
    inline void clear_list() {
        cloud_path_.clear();
        cloud_name_.clear();
        refresh_list();
    }

    inline int get_cloud_index() {
        return page_.list_cloud->currentRow();
    }
};

} // namespace workspace