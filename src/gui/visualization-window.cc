#include "visualization-window.hh"
#include "utility/qt_binding.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/widget/cards/filled-card.hh>

using namespace creeper;

template <class T>
struct Rounded : public T {
    using T::T;

    auto resizeEvent(QResizeEvent* event) -> void override {
        T::resizeEvent(event);

        auto path = QPainterPath {};
        path.addRoundedRect(this->rect(), rounded_radius, rounded_radius);

        auto mask = QRegion(path.toFillPolygon().toPolygon());
        T::setMask(mask);
    }

    double rounded_radius = 10;
};

auto VisualizationWindowComponent(VisualizationWindowState& state) noexcept -> QPointer<QWidget> {

    const auto QtVtkWindowComponent = [&] {
        auto window = new Rounded<pcs::QtVtkWindow> {};

        state.renderer.connect_ui(*window);

        return window;
    };

    return new FilledCard {
        card::pro::ThemeManager { state.manager },
        card::pro::LevelDefault,
        card::pro::Radius { 0 },
        card::pro::Layout<Row> {
            row::pro::Margin { 10 },
            row::pro::Item { QtVtkWindowComponent() },
        },
    };
}
