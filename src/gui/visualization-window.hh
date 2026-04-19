#pragma once
#include "core/renderer.hh"
#include "gui/interaction/mouse.hh"

#include <creeper-qt/utility/theme/theme.hh>

#include <qpointer.h>
#include <qwidget.h>

struct VisualizationWindowState {
    creeper::ThemeManager& manager;
    pcs::Renderer& renderer;
    pcs::gui::interaction::Mouse* mouse = nullptr;
};

auto VisualizationWindowComponent(VisualizationWindowState&) noexcept -> QPointer<QWidget>;
