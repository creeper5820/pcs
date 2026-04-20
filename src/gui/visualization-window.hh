#pragma once
#include "core/renderer.hh"
#include "core/runtime.hh"
#include "gui/interaction/mouse.hh"

#include <creeper-qt/utility/theme/theme.hh>

#include <qpointer.h>
#include <qwidget.h>

struct VisualizationWindowState {
    creeper::ThemeManager& manager;
    pcs::Renderer& renderer;
    pcs::Runtime* runtime = nullptr;
    pcs::gui::interaction::Mouse* mouse = nullptr;
};

auto VisualizationWindowComponent(VisualizationWindowState&) noexcept -> QPointer<QWidget>;
