#pragma once

#include "core/renderer.hh"
#include "core/runtime.hh"
#include "gui/interaction/mouse.hh"

#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/utility/wrapper/pimpl.hh>

#include <QWidget>

class VisualizationWindow final : public QWidget {
    CREEPER_PIMPL_DEFINITION(VisualizationWindow)

public:
    VisualizationWindow(creeper::ThemeManager& manager, pcs::Renderer& renderer,
        pcs::Runtime& runtime, pcs::gui::interaction::Mouse& mouse) noexcept;
};
