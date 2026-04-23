#include "visualization-window.hh"
#include "utility/qt_binding.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/text.hh>

#include <QFont>
#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainterPath>
#include <QPointer>
#include <QRegion>
#include <QVBoxLayout>

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <algorithm>
#include <tuple>

using namespace creeper;

template <class T>
struct Rounded : public T {
    using T::T;

    auto resizeEvent(QResizeEvent* event) -> void override {
        T::resizeEvent(event);

        auto path = QPainterPath { };
        path.addRoundedRect(this->rect(), rounded_radius, rounded_radius);

        auto mask = QRegion(path.toFillPolygon().toPolygon());
        T::setMask(mask);
    }

    double rounded_radius = 10;
};

class InteractiveVtkWindow : public Rounded<pcs::QtVtkWindow> {
public:
    explicit InteractiveVtkWindow(
        ThemeManager& manager, pcs::gui::interaction::Mouse& mouse) noexcept
        : mouse { mouse } {
        setMouseTracking(true);

        auto font = QFont { "WenQuanYi Micro Hei", 10 };
        font.setStyleHint(QFont::SansSerif);

        const auto overlay_text_props = std::tuple {
            text::pro::ThemeManager { manager },
            text::pro::Font { font },
            text::pro::Alignment { Qt::AlignLeft | Qt::AlignVCenter },
            text::pro::Apply { [](Text& text) {
                text.setAttribute(Qt::WA_TransparentForMouseEvents, true);
                text.setMargin(6);
            } },
        };

        const auto overlay_card_props = std::tuple {
            card::pro::ThemeManager { manager },
            widget::pro::Parent { this },
            card::pro::Radius { 8 },
            card::pro::LevelLow,
            widget::pro::Apply { [](QWidget& widget) {
                widget.setAttribute(Qt::WA_TransparentForMouseEvents, true);
            } },
        };

        status_label = new Text { overlay_text_props };
        status_card  = new FilledCard {
            overlay_card_props,
            card::pro::Layout<Row> {
                row::pro::Margin { 0 },
                row::pro::Item { status_label },
            },
        };
        status_card->show();

        operation_label = new Text { overlay_text_props };
        operation_card  = new FilledCard {
            overlay_card_props,
            card::pro::Layout<Row> {
                row::pro::Margin { 0 },
                row::pro::Item { operation_label },
            },
        };
        operation_card->hide();

        version_label = new Text {
            overlay_text_props,
            text::pro::Text { QString::fromUtf8(APP_VERSION) },
        };
        version_card = new FilledCard {
            overlay_card_props,
            card::pro::Layout<Row> {
                row::pro::Margin { 0 },
                row::pro::Item { version_label },
            },
        };
        version_card->show();

        update_overlay_labels();
    }

    auto set_status_text(QString text) noexcept -> void {
        status_text = std::move(text);
        update_overlay_labels();
    }

    auto set_operation_text(QString text) noexcept -> void {
        operation_text = std::move(text);
        update_overlay_labels();
    }

    auto sync_interaction_state() noexcept -> void {
        auto* interactor = renderWindow() != nullptr ? renderWindow()->GetInteractor() : nullptr;
        if (interactor == nullptr) {
            return;
        }

        if (allow_camera_interaction()) {
            interactor->Enable();
            return;
        }

        interactor->Disable();
    }

protected:
    auto resizeEvent(QResizeEvent* event) -> void override {
        Rounded<pcs::QtVtkWindow>::resizeEvent(event);
        update_overlay_labels();
    }

    auto mouseMoveEvent(QMouseEvent* event) -> void override {
        emit_move(*event);
        if (allow_camera_interaction()) {
            Rounded<pcs::QtVtkWindow>::mouseMoveEvent(event);
            return;
        }

        event->accept();
    }

    auto mousePressEvent(QMouseEvent* event) -> void override {
        emit_click(*event);
        if (allow_camera_interaction()) {
            Rounded<pcs::QtVtkWindow>::mousePressEvent(event);
            return;
        }

        event->accept();
    }

    auto mouseReleaseEvent(QMouseEvent* event) -> void override {
        if (allow_camera_interaction()) {
            Rounded<pcs::QtVtkWindow>::mouseReleaseEvent(event);
            return;
        }

        event->accept();
    }

private:
    auto allow_camera_interaction() const noexcept -> bool {
        return mouse.allows_camera_interaction();
    }

    auto update_overlay_labels() noexcept -> void {
        constexpr auto kMargin = 12;
        constexpr auto kGap    = 8;

        version_label->setText(QString::fromUtf8(APP_VERSION));
        version_card->adjustSize();

        const auto version_w = version_card->width();
        const auto version_h = version_card->height();
        const auto version_x = std::max(0, width() - kMargin - version_w);
        const auto version_y = std::max(0, height() - kMargin - version_h);
        version_card->move(version_x, version_y);

        if (status_text.isEmpty()) {
            status_card->hide();
        } else {
            status_card->show();

            const auto max_text_width = std::max(120, width() / 2 - kMargin * 2);
            auto metrics              = QFontMetrics { status_label->font() };
            auto text = metrics.elidedText(status_text, Qt::ElideRight, max_text_width);
            status_label->setText(text);
            status_card->adjustSize();

            const auto h = status_card->height();
            status_card->move(kMargin, std::max(0, height() - kMargin - h));
        }

        if (operation_text.isEmpty()) {
            operation_card->hide();
        } else {
            operation_card->show();

            const auto max_text_width = std::max(120, width() - kMargin * 2 - version_w - kGap);
            auto metrics              = QFontMetrics { operation_label->font() };
            auto text = metrics.elidedText(operation_text, Qt::ElideRight, max_text_width);
            operation_label->setText(text);
            operation_card->adjustSize();

            const auto w = operation_card->width();
            const auto h = operation_card->height();
            const auto x = std::max(0, version_x - kGap - w);
            const auto y = std::max(0, height() - kMargin - h);
            operation_card->move(x, y);
        }
    }

    auto emit_move(QMouseEvent const& event) noexcept -> void {
        mouse.emit_move(pcs::gui::interaction::MouseEvent {
            .x         = event.position().toPoint().x(),
            .y         = event.position().toPoint().y(),
            .modifiers = event.modifiers(),
            .buttons   = event.buttons(),
        });
    }

    auto emit_click(QMouseEvent const& event) noexcept -> void {
        const auto payload = pcs::gui::interaction::MouseEvent {
            .x         = event.position().toPoint().x(),
            .y         = event.position().toPoint().y(),
            .modifiers = event.modifiers(),
            .buttons   = event.buttons(),
        };

        if (event.button() == Qt::LeftButton) {
            mouse.emit_lclick(payload);
            return;
        }
        if (event.button() == Qt::RightButton) {
            mouse.emit_rclick(payload);
            return;
        }
    }

    pcs::gui::interaction::Mouse& mouse;
    FilledCard* status_card    = nullptr;
    FilledCard* operation_card = nullptr;
    FilledCard* version_card   = nullptr;
    Text* status_label         = nullptr;
    Text* operation_label      = nullptr;
    Text* version_label        = nullptr;
    QString status_text;
    QString operation_text;
};

struct VisualizationWindow::Impl {
    InteractiveVtkWindow* window = nullptr;
};

VisualizationWindow::VisualizationWindow(ThemeManager& manager, pcs::Renderer& renderer,
    pcs::Runtime& runtime, pcs::gui::interaction::Mouse& mouse) noexcept
    : pimpl { std::make_unique<Impl>() } {
    pimpl->window = new InteractiveVtkWindow { manager, mouse };

    renderer.connect_ui(*pimpl->window);
    pimpl->window->sync_interaction_state();

    mouse.set_status_sink(
        [guard = QPointer<InteractiveVtkWindow> { pimpl->window }](QString const& text) {
            if (guard != nullptr) {
                guard->set_status_text(text);
            }
        });
    mouse.set_mode_sink([guard = QPointer<InteractiveVtkWindow> { pimpl->window }](auto) {
        if (guard != nullptr) {
            guard->sync_interaction_state();
        }
    });
    mouse.set_png_edit_tool_sink([guard = QPointer<InteractiveVtkWindow> { pimpl->window }](auto) {
        if (guard != nullptr) {
            guard->sync_interaction_state();
        }
    });

    runtime.set_operation_sink(
        [guard = QPointer<InteractiveVtkWindow> { pimpl->window }](std::string const& message) {
            if (guard != nullptr) {
                guard->set_operation_text(QString::fromStdString(message));
            }
        });

    auto* root = new FilledCard {
        card::pro::ThemeManager { manager },
        card::pro::LevelDefault,
        card::pro::Radius { 0 },
        card::pro::Layout<Row> {
            row::pro::Margin { 10 },
            row::pro::Item { pimpl->window },
        },
    };

    auto* layout = new QVBoxLayout { };
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(root);
    setLayout(layout);
}

VisualizationWindow::~VisualizationWindow() = default;
