#include "visualization-window.hh"
#include "utility/qt_binding.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/widget/cards/filled-card.hh>

#include <QFont>
#include <QFontMetrics>
#include <QLabel>
#include <QMouseEvent>
#include <QPainterPath>
#include <QPointer>
#include <QRegion>
#include <QVBoxLayout>

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <algorithm>

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
    explicit InteractiveVtkWindow(pcs::gui::interaction::Mouse& mouse) noexcept
        : mouse { mouse } {
        setMouseTracking(true);

        status_label = new QLabel { this };
        status_label->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        status_label->setStyleSheet("QLabel {"
                                    "color: white;"
                                    "background: rgba(0, 0, 0, 160);"
                                    "padding: 6px 8px;"
                                    "border-radius: 8px;"
                                    "}");

        auto font = QFont { "WenQuanYi Micro Hei", 10 };
        font.setStyleHint(QFont::SansSerif);
        status_label->setFont(font);
        status_label->show();

        operation_label = new QLabel { this };
        operation_label->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        operation_label->setStyleSheet("QLabel {"
                                       "color: white;"
                                       "background: rgba(0, 0, 0, 160);"
                                       "padding: 6px 8px;"
                                       "border-radius: 8px;"
                                       "}");
        operation_label->setFont(font);
        operation_label->hide();
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
        if (status_label == nullptr || operation_label == nullptr) {
            return;
        }

        constexpr auto kMargin = 12;

        if (status_text.isEmpty()) {
            status_label->hide();
        } else {
            status_label->show();

            const auto max_text_width = std::max(120, width() / 2 - kMargin * 2);
            auto metrics              = QFontMetrics { status_label->font() };
            auto text = metrics.elidedText(status_text, Qt::ElideRight, max_text_width);
            status_label->setText(text);
            status_label->adjustSize();

            const auto h = status_label->height();
            status_label->move(kMargin, std::max(0, height() - kMargin - h));
        }

        if (operation_text.isEmpty()) {
            operation_label->hide();
        } else {
            operation_label->show();

            const auto max_text_width = std::max(120, width() / 2 - kMargin * 2);
            auto metrics               = QFontMetrics { operation_label->font() };
            auto text = metrics.elidedText(operation_text, Qt::ElideRight, max_text_width);
            operation_label->setText(text);
            operation_label->adjustSize();

            const auto w = operation_label->width();
            const auto h = operation_label->height();
            operation_label->move(
                std::max(0, width() - kMargin - w), std::max(0, height() - kMargin - h));
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
    QLabel* status_label                = nullptr;
    QLabel* operation_label             = nullptr;
    QString status_text;
    QString operation_text;
};

struct VisualizationWindow::Impl {
    InteractiveVtkWindow* window = nullptr;
};

VisualizationWindow::VisualizationWindow(ThemeManager& manager, pcs::Renderer& renderer,
    pcs::Runtime& runtime, pcs::gui::interaction::Mouse& mouse) noexcept
    : pimpl { std::make_unique<Impl>() } {
    pimpl->window = new InteractiveVtkWindow { mouse };

    renderer.connect_ui(*pimpl->window);
    pimpl->window->sync_interaction_state();

    mouse.set_status_sink([guard = QPointer<InteractiveVtkWindow> { pimpl->window }](
                              QString const& text) {
        if (guard != nullptr) {
            guard->set_status_text(text);
        }
    });
    mouse.set_mode_sink([guard = QPointer<InteractiveVtkWindow> { pimpl->window }](auto) {
        if (guard != nullptr) {
            guard->sync_interaction_state();
        }
    });
    mouse.set_png_edit_tool_sink(
        [guard = QPointer<InteractiveVtkWindow> { pimpl->window }](auto) {
            if (guard != nullptr) {
                guard->sync_interaction_state();
            }
        });

    runtime.set_operation_sink([guard = QPointer<InteractiveVtkWindow> { pimpl->window }](
                                  std::string const& message) {
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

VisualizationWindow::~VisualizationWindow() noexcept = default;
