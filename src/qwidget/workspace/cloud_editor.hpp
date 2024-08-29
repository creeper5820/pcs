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
#include <ui_cloud_editor.h>

#include <regex>

namespace workspace {
class CloudEditor : public QWidget, Ui::CloudEditor {
    Q_OBJECT
public:
    CloudEditor(QWidget* parent = nullptr)
        : QWidget(parent) {
        setupUi(this);

        // bind icon with button
        button_clear->setIcon(QPixmap(":pic/clear.svg"));
        button_load->setIcon(QPixmap(":pic/file.svg"));
        button_refresh->setIcon(QPixmap(":pic/reset.svg"));

        // custom context menu
        list_cloud->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(list_cloud, &QWidget::customContextMenuRequested,
            this, &CloudEditor::display_menu_of_list);

        // connect
        connect(button_refresh, &QPushButton::clicked,
            this, &CloudEditor::refresh);
        connect(button_load, &QPushButton::clicked,
            this, &CloudEditor::load_cloud);
        connect(button_clear, &QPushButton::clicked,
            this, &CloudEditor::clear_cloud);
        connect(list_cloud, &QListWidget::itemClicked,
            this, &CloudEditor::update_cloud_editor);
        connect(editor_cloud_name, &QLineEdit::editingFinished,
            this, &CloudEditor::update_cloud_name);
        connect(editor_cloud_color, &QLineEdit::editingFinished,
            this, &CloudEditor::update_cloud_color);
    }
private slots:
    void remove_current_cloud() {
        const auto index = get_cloud_index();
        if (index == -1)
            return;
        core::View::instance().remove_cloud(paths_[index].toStdString());
        remove_from_list(index);
    }

    void display_menu_of_list() {
        if (get_cloud_index() == -1)
            return;
        auto menu = QMenu {};
        menu.addAction("delete", this,
            &CloudEditor::remove_current_cloud);
        menu.exec(QCursor::pos());
    }

    void update_cloud_editor() {
        const auto index = get_cloud_index();
        if (index == -1) {
            editor_cloud_name->setText("");
            editor_cloud_color->setText("");
            editor_cloud_frame->setText("");
            return;
        }
        const auto path = paths_[index].toStdString();
        auto cloud = core::View::instance()[path];

        const auto name = names_[index];
        const auto frame = QString::fromStdString(cloud->frame());
        const auto rgb = cloud->color();
        const auto color = QString::asprintf(
            "%d, %d, %d", rgb.r, rgb.g, rgb.b);

        editor_cloud_name->setText(name);
        editor_cloud_color->setText(color);
        editor_cloud_frame->setText(frame);
    }

    void update_cloud_name() {
        auto index = get_cloud_index();
        if (index == -1)
            return;

        auto input = editor_cloud_name->text();
        auto target = input.contains(".")
            ? input
            : input + ".pcd";

        names_[index] = target;
        refresh_list();
    }

    void update_cloud_color() {
        static const auto regex_rgb
            = std::regex("(\\d{1,3})\\s*,\\s*(\\d{1,3})\\s*,\\s*(\\d{1,3})");
        static const auto regex_hex
            = std::regex("^\\s*#[0-9|a-f|A-F]{6}\\s*$");

        auto index = get_cloud_index();
        if (index == -1)
            return;

        auto text = editor_cloud_color->text().toStdString();
        const auto path = paths_[index].toStdString();
        auto match = std::smatch();
        if (std::regex_match(text, match, regex_rgb)) {
            core::View::instance().set_color(path,
                std::stoi(match[1]),
                std::stoi(match[2]),
                std::stoi(match[3]));
        } else if (std::regex_match(text, match, regex_hex)) {
        } else {
            util::message("wrong color");
        }
    }

    void refresh() {
        core::View::instance().render();
        core::View::instance().reset_camera();
    }

    void load_cloud() {
        const auto _path = QFileDialog::getOpenFileName(
            this, "Open File", "", "PCD Files (*.pcd)");
        const auto path = _path.toStdString();
        const auto name = path.substr(path.find_last_of("/") + 1);
        if (!_path.isEmpty() && !paths_.contains(_path)
            && core::View::instance().load_cloud(path))
            append_to_list(_path);
    }

    void clear_cloud() {
        core::View::instance().clear_cloud();
        clear_list();
    }

private:
    QStringList paths_;
    QStringList names_;

    static inline auto get_name_form_path(const QString& path) {
        return path.right(path.size() - path.lastIndexOf("/") - 1);
    }

    // TODO: fuck this
    inline void refresh_list() {
        list_cloud->clear();
        list_cloud->addItems(names_);
    }
    inline void append_to_list(const QString& path) {
        paths_.append(path);
        names_.append(get_name_form_path(path));
        refresh_list();
    }
    inline void remove_from_list(int index) {
        paths_.removeAt(index);
        names_.removeAt(index);
        refresh_list();
    }
    inline void remove_from_list(const QString& path) {
        paths_.removeOne(path);
        names_.removeOne(get_name_form_path(path));
        refresh_list();
    }
    inline void clear_list() {
        paths_.clear();
        names_.clear();
        refresh_list();
    }

    inline int get_cloud_index() {
        return list_cloud->currentRow();
    }
};

} // namespace workspace