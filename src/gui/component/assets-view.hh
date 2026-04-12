#pragma once
#include "core/assets.hh"

#include <creeper-qt/layout/group.hh>
#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/cards/outlined-card.hh>
#include <creeper-qt/widget/text.hh>
#include <creeper-qt/widget/widget.hh>

#include <QSignalBlocker>
#include <qfileinfo.h>
#include <qlistview.h>
#include <qstringlistmodel.h>
#include <qstyleditemdelegate.h>

struct AssetsView : public creeper::FilledCard {

    creeper::ThemeManager& theme;
    pcs::AssetsManager& assets;

    QStringListModel& string_list;
    std::function<void(std::string const&)> selection_callback;
    QListView* list_view = nullptr;

    explicit AssetsView(auto& theme, auto& assets, auto& string_list, auto f) noexcept
        : theme { theme }
        , assets { assets }
        , string_list { string_list }
        , selection_callback { f } {

        using namespace creeper;
        auto theme_prop = theme::pro::ThemeManager { theme };
        auto font       = QFont { "WenQuanYi Micro Hei Mono", 10 };

        struct AssetDelegate : public QStyledItemDelegate {
            pcs::AssetsManager& assets;

            explicit AssetDelegate(pcs::AssetsManager& assets)
                : assets { assets } { }

            QString displayText(const QVariant& value, const QLocale&) const override {
                auto id = value.toString().toStdString();
                if (auto display = assets.get_asset_display_name(id)) {
                    return QString::fromStdString(*display);
                }
                return value.toString();
            }
        };

        list_view = new QListView;
        list_view->setModel(&string_list);
        list_view->setItemDelegate(new AssetDelegate(assets));
        list_view->setStyleSheet(R"(
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
        QObject::connect(list_view, &QListView::clicked, //
            [this](const QModelIndex& index) {
                const auto model = list_view->model();
                const auto data  = model->data(index, Qt::DisplayRole);
                selection_callback(data.toString().toStdString());
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
                col::pro::Item { list_view },
                col::pro::Stretch { 255 },
            },
        };
        FilledCard::apply(props);
    }

    auto select_asset(std::string const& id) noexcept -> void {
        const auto ids = string_list.stringList();
        const auto key = QString::fromStdString(id);

        for (int row = 0; row < ids.size(); ++row) {
            if (ids[row] != key) {
                continue;
            }

            const auto index   = string_list.index(row);
            const auto blocker = QSignalBlocker { list_view };
            list_view->setCurrentIndex(index);
            selection_callback(id);
            return;
        }
    }
};
