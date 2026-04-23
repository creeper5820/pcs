#include "vtk-window.hh"
#include "utility/qt_binding.hh"

#include <qpainterpath.h>
#include <qtimer.h>

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/widget/cards/filled-card.hh>

using namespace pcs;
using namespace creeper;

constexpr auto kVtkWindowBorderRadius = double { 10 };
constexpr auto kVtkWindowBorderWidth  = double { 10 };

class VtkWidget : public pcs::QtVtkWindow {

public:
    explicit VtkWidget() noexcept
        : pcs::QtVtkWindow { } {
        connect(&resize_watch_dog, &QTimer::timeout, [this] { setVisible(true); });
    }

    void set_background(const QColor& color) { background = color; }

protected:
    void resizeEvent(QResizeEvent* e) override {
        setVisible(false);
        using namespace std::chrono_literals;
        resize_watch_dog.start(100ms);
        QVTKOpenGLNativeWidget::resizeEvent(e);
    }

    void paintEvent(QPaintEvent* e) override {
        // 先绘制 VTK 内容，再叠加圆角边框。
        QVTKOpenGLNativeWidget::paintEvent(e);
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen { background, kVtkWindowBorderWidth });
        p.drawRoundedRect(rect(), kVtkWindowBorderRadius, kVtkWindowBorderRadius);
    }

    void mousePressEvent(QMouseEvent* e) override { QVTKOpenGLNativeWidget::mousePressEvent(e); }

private:
    QTimer resize_watch_dog;
    QColor background;
};

struct VtkWindow::Impl {

    VtkWidget* vtk_widget  = nullptr;
    FilledCard* background = nullptr;

    explicit Impl(ThemeManager& manager) noexcept {

        vtk_widget = new VtkWidget { };

        namespace c = card::pro;
        namespace l = linear::pro;
        background  = new FilledCard {
            c::Radius { 15 },
            c::BorderWidth { 2 },
            c::Layout<Col> {
                l::Margin { 8 },
                l::Spacing { 0 },
                l::Item { vtk_widget },
            },
        };

        manager.append_handler(background, [this](const ThemeManager& manager) {
            const auto scheme = manager.color_scheme();
            const auto color  = scheme.surface_container_highest;
            background->set_background(color);
            background->set_border_color(scheme.secondary_container);
            vtk_widget->set_background(color);
        });
    }
};

VtkWindow::VtkWindow(ThemeManager& manager)
    : pimpl(std::make_unique<Impl>(manager)) { }

VtkWindow::~VtkWindow() = default;

auto VtkWindow::component() -> QWidget* const { return pimpl->background; }
