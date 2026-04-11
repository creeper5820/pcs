#pragma once
#include "core/assets.hh"

#include <creeper-qt/layout/group.hh>
#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/cards/outlined-card.hh>
#include <creeper-qt/widget/text.hh>
#include <creeper-qt/widget/widget.hh>

#include <qfileinfo.h>
#include <qlistview.h>
#include <qstringlistmodel.h>
#include <qstyleditemdelegate.h>

struct AssetsView : public creeper::FilledCard {

    creeper::ThemeManager& theme;
    pcs::AssetsManager& assets;

    QStringListModel& string_list;
    std::function<void(std::string_view)> selection_callback;

    explicit AssetsView(auto& theme, auto& assets, auto& string_list, auto f) noexcept
        : theme { theme }
        , assets { assets }
        , string_list { string_list }
        , selection_callback { f } {

        using namespace creeper;
        auto theme_prop = theme::pro::ThemeManager { theme };
        auto font       = QFont { "WenQuanYi Micro Hei Mono", 10 };

        struct FilenameDelegate : public QStyledItemDelegate {
            using QStyledItemDelegate::QStyledItemDelegate;
            QString displayText(const QVariant& value, const QLocale&) const override {
                return QFileInfo(value.toString()).fileName();
            }
        };

        auto NativeListView = new QListView;
        NativeListView->setModel(&string_list);
        NativeListView->setItemDelegate(new FilenameDelegate);
        NativeListView->setStyleSheet(R"(
            QListView {
                font: 10pt "WenQuanYi Micro Hei";
                border: 0px solid #cccccc;
                border-radius: 5px;
                background-color: #f9f9f9;
                padding: 2px;
            }
            QListView::item {
                height: 20px;
                padding-left: 5px;
                color: #333333;
            }
            QListView::item:hover {
                background-color: #e6f7ff;
            }
            QListView::item:selected {
                background-color: #bae7ff;
                border-left: 4px solid #1890ff;
                color: #000000;
            }
        )");
        QObject::connect(NativeListView, &QListView::clicked, //
            [this, NativeListView](const QModelIndex& index) {
                const auto model   = NativeListView->model();
                const auto data    = model->data(index, Qt::DisplayRole);
                const auto str     = data.toString();
                const auto std_str = str.toStdString();
                const auto view    = std::string_view { std_str };
                selection_callback(view);
            });

        auto props = std::tuple {
            card::pro::ThemeManager { theme },
            card::pro::LevelLowest,
            card::pro::Layout<Col> {
                col::pro::Margin { 10 },
                col::pro::Spacing { 10 },
                col::pro::Item<Text> {
                    theme_prop,
                    text::pro::Font { font },
                    text::pro::Text { "Assets View" },
                    text::pro::Alignment { Qt::AlignHCenter },
                },
                col::pro::Item { NativeListView },
                col::pro::Stretch { 255 },
            },
        };
        FilledCard::apply(props);
    }
};
