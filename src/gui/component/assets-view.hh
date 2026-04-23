#pragma once
#include "core/assets.hh"

#include <creeper-qt/layout/group.hh>
#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/utility/material-icon.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/cards/outlined-card.hh>
#include <creeper-qt/widget/buttons/icon-button.hh>
#include <creeper-qt/widget/text.hh>
#include <creeper-qt/widget/widget.hh>

#include <QVBoxLayout>
#include <QMouseEvent>
#include <qframe.h>
#include <qscrollarea.h>
#include <qstringlistmodel.h>

#include <functional>
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

struct AssetsView : public creeper::FilledCard {

    struct AssetRowWidget final : public QFrame {
        creeper::ThemeManager& theme;
        pcs::AssetsManager& assets;
        std::string asset_id;
        std::function<void(std::string const&)> select_callback;
        std::function<void(std::string const&)> toggle_visibility_callback;
        std::function<void(std::string const&)> delete_callback;
        creeper::IconButton* visibility_button = nullptr;
        creeper::Text* name_text              = nullptr;
        creeper::Text* type_text              = nullptr;
        bool selected                         = false;

        explicit AssetRowWidget(creeper::ThemeManager& theme, pcs::AssetsManager& assets,
            std::string id, auto on_select, auto on_toggle_visibility, auto on_delete) noexcept
            : theme { theme }
            , assets { assets }
            , asset_id { std::move(id) }
            , select_callback { std::move(on_select) }
            , toggle_visibility_callback { std::move(on_toggle_visibility) }
            , delete_callback { std::move(on_delete) } {
            using namespace creeper;

            setObjectName("assetRow");
            setCursor(Qt::PointingHandCursor);
            setFrameStyle(QFrame::NoFrame);

            const auto font = QFont { "WenQuanYi Micro Hei Mono", 10 };
            const auto icon_size = [] {
                const auto base  = IconButton::kSmallContainerSize;
                const auto width = std::max(16, static_cast<int>(std::round(base.width() * 2.0 / 3.0)));
                const auto height = std::max(16, static_cast<int>(std::round(base.height() * 2.0 / 3.0)));
                return QSize { width, height };
            }();
            const auto icon_font_size =
                std::max(10, static_cast<int>(std::round(IconButton::kSmallFontIconSize * 2.0 / 3.0)));

            visibility_button = new IconButton {
                icon_button::pro::ThemeManager { theme },
                icon_button::pro::FixedSize { icon_size },
                icon_button::pro::Font { material::round::font, icon_font_size },
                icon_button::pro::ShapeSquare,
                icon_button::pro::ColorStandard,
                icon_button::pro::ToolTip { "切换资产可见性" },
                icon_button::pro::Clickable { [this] {
                    if (select_callback) {
                        select_callback(asset_id);
                    }
                    if (toggle_visibility_callback) {
                        toggle_visibility_callback(asset_id);
                    }
                } },
            };

            name_text = new Text {
                text::pro::ThemeManager { theme },
                text::pro::Font { font },
                text::pro::Alignment { Qt::AlignVCenter | Qt::AlignLeft },
            };

            type_text = new Text {
                text::pro::ThemeManager { theme },
                text::pro::Font { font },
                text::pro::Alignment { Qt::AlignVCenter | Qt::AlignRight },
            };

            auto* delete_button = new IconButton {
                icon_button::pro::ThemeManager { theme },
                icon_button::pro::FixedSize { icon_size },
                icon_button::pro::Font { material::round::font, icon_font_size },
                icon_button::pro::FontIcon { "delete" },
                icon_button::pro::ShapeSquare,
                icon_button::pro::ColorStandard,
                icon_button::pro::ToolTip { "删除当前资产" },
                icon_button::pro::Clickable { [this] {
                    if (select_callback) {
                        select_callback(asset_id);
                    }
                    if (delete_callback) {
                        delete_callback(asset_id);
                    }
                } },
            };

            auto* row = new Row {
                row::pro::Spacing { 8 },
                row::pro::Margin { 8 },
                row::pro::Alignment { Qt::AlignVCenter },
                row::pro::Item { visibility_button },
                row::pro::Item { { 0, Qt::AlignVCenter }, name_text },
                row::pro::Stretch { 255 },
                row::pro::Item { { 0, Qt::AlignVCenter }, type_text },
                row::pro::Item { delete_button },
            };
            row->setContentsMargins(0, 0, 0, 0);
            setLayout(row);

            refresh();
            apply_style(theme);
            theme.append_handler(this, [this](const ThemeManager& manager) { apply_style(manager); });
            theme.apply_theme();
        }

        auto refresh() noexcept -> void {
            const auto asset_name = assets.get_asset_name(asset_id);
            name_text->setText(QString::fromStdString(asset_name));
            name_text->setToolTip(QString::fromStdString(asset_name));

            const auto asset_type = [this]() -> QString {
                return QString::fromStdString(std::string { assets.get_asset_kind(asset_id) });
            }();

            type_text->setText(asset_type);
            type_text->setToolTip(asset_type);

            const auto visible = assets.is_asset_visible(asset_id);
            visibility_button->set_selected(!visible);
            visibility_button->set_icon(visible ? "visibility" : "visibility_off");
            visibility_button->setToolTip(visible ? "当前可见" : "当前隐藏");
            visibility_button->update();
        }

        auto set_selected(bool on) noexcept -> void {
            selected = on;
            apply_style(theme);
        }

    private:
        auto apply_style(creeper::ThemeManager const& manager) noexcept -> void {
            using namespace creeper;
            const auto border = selected
                ? (manager.color_mode() == ColorMode::DARK ? "#6ca5ff" : "#1890ff")
                : (manager.color_mode() == ColorMode::DARK ? "#2f3a46" : "#d5dce5");
            const auto background = selected
                ? (manager.color_mode() == ColorMode::DARK ? "#24384f" : "#e6f7ff")
                : "transparent";
            const auto hover = manager.color_mode() == ColorMode::DARK ? "#2b3440" : "#f5f9ff";

            setStyleSheet(QString(R"(
                QFrame#assetRow {
                    border: 1px solid %1;
                    border-radius: 10px;
                    background-color: %2;
                }
                QFrame#assetRow:hover {
                    background-color: %3;
                }
            )")
                              .arg(border, background, selected ? background : hover));
        }

    protected:
        void mousePressEvent(QMouseEvent* event) override {
            if (event->button() == Qt::LeftButton && select_callback) {
                select_callback(asset_id);
            }
            QFrame::mousePressEvent(event);
        }
    };

    creeper::ThemeManager& theme;
    pcs::AssetsManager& assets;

    QStringListModel& string_list;
    std::function<void(std::string const&)> selection_callback;
    std::function<void(std::string const&)> toggle_visibility_callback;
    std::function<void(std::string const&)> delete_callback;
    QScrollArea* scroll_area = nullptr;
    QWidget* list_container  = nullptr;
    QVBoxLayout* list_layout = nullptr;
    std::optional<std::string> selected_asset_id;
    std::vector<AssetRowWidget*> asset_rows;

    explicit AssetsView(auto& theme, auto& assets, auto& string_list, auto on_select,
        auto on_toggle_visibility, auto on_delete) noexcept
        : theme { theme }
        , assets { assets }
        , string_list { string_list }
        , selection_callback { std::move(on_select) }
        , toggle_visibility_callback { std::move(on_toggle_visibility) }
        , delete_callback { std::move(on_delete) } {

        using namespace creeper;
        auto theme_prop = theme::pro::ThemeManager { theme };
        auto font       = QFont { "WenQuanYi Micro Hei Mono", 10 };

        list_container = new QWidget;
        list_layout    = new QVBoxLayout;
        list_layout->setContentsMargins(0, 0, 0, 0);
        list_layout->setSpacing(8);
        list_container->setLayout(list_layout);

        scroll_area = new QScrollArea;
        scroll_area->setWidgetResizable(true);
        scroll_area->setFrameShape(QFrame::NoFrame);
        scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scroll_area->setWidget(list_container);
        scroll_area->setMinimumHeight(360);

        const auto sync_scroll_qss = [this](const ThemeManager& manager) {
            const auto background = manager.color_mode() == ColorMode::DARK ? "#1f242b" : "#f9f9f9";
            list_container->setStyleSheet("background: transparent;");
            scroll_area->setStyleSheet(QString("QScrollArea { border: none; background-color: %1; }").arg(background));
        };

        sync_scroll_qss(theme);
        theme.append_handler(scroll_area, sync_scroll_qss);

        const auto rebuild_rows = [this] {
            while (list_layout->count() > 0) {
                auto* item = list_layout->takeAt(0);
                if (auto* widget = item->widget(); widget != nullptr) {
                    widget->deleteLater();
                }
                delete item;
            }

            asset_rows.clear();

            const auto ids = this->string_list.stringList();
            for (const auto& id_text : ids) {
                auto* row = new AssetRowWidget(this->theme, this->assets, id_text.toStdString(),
                    [this](std::string const& id) { select_asset(id); },
                    [this](std::string const& id) {
                        if (toggle_visibility_callback) {
                            toggle_visibility_callback(id);
                        }
                        refresh_row(id);
                    },
                    [this](std::string const& id) {
                        if (delete_callback) {
                            delete_callback(id);
                        }
                    });
                asset_rows.push_back(row);
                list_layout->addWidget(row);
            }

            list_layout->addStretch(1);

            if (selected_asset_id.has_value()) {
                set_selected_row(*selected_asset_id);
            }

            this->theme.apply_theme();
        };

        QObject::connect(&string_list, &QStringListModel::modelReset, this,
            [rebuild_rows] { rebuild_rows(); });
        QObject::connect(&string_list, &QStringListModel::rowsInserted, this,
            [rebuild_rows](const QModelIndex&, int, int) { rebuild_rows(); });
        QObject::connect(&string_list, &QStringListModel::rowsRemoved, this,
            [rebuild_rows](const QModelIndex&, int, int) { rebuild_rows(); });
        QObject::connect(&string_list, &QStringListModel::dataChanged, this,
            [rebuild_rows](const QModelIndex&, const QModelIndex&, const QList<int>&) { rebuild_rows(); });

        rebuild_rows();

        auto props = std::tuple {
            card::pro::ThemeManager { theme },
            card::pro::LevelLowest,
            card::pro::Layout<Col> {
                col::pro::Margin { 10 },
                col::pro::Spacing { 10 },
                col::pro::Item<Text> {
                    theme_prop,
                    text::pro::Font { font },
                    text::pro::Text { "资产列表" },
                    text::pro::Alignment { Qt::AlignHCenter },
                },
                col::pro::Item { scroll_area },
                col::pro::Stretch { 255 },
            },
        };
        FilledCard::apply(props);
    }

    auto select_asset(std::string const& id) noexcept -> void {
        set_selected_row(id);
        selection_callback(id);
    }

    auto refresh_row(std::string const& id) noexcept -> void {
        for (auto* row : asset_rows) {
            if (row != nullptr && row->asset_id == id) {
                row->refresh();
                return;
            }
        }
    }

    auto clear_selection() noexcept -> void {
        selected_asset_id.reset();
        set_selected_row({ });
    }

private:
    auto set_selected_row(std::string const& id) noexcept -> void {
        if (id.empty()) {
            for (auto* row : asset_rows) {
                if (row != nullptr) {
                    row->set_selected(false);
                }
            }
            return;
        }

        selected_asset_id = id;
        for (auto* row : asset_rows) {
            if (row != nullptr) {
                row->set_selected(row->asset_id == id);
            }
        }
    }
};
