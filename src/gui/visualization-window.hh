#pragma once
#include "core/renderer.hh"

#include <creeper-qt/utility/theme/theme.hh>

#include <qpointer.h>
#include <qwidget.h>

struct VisualizationWindowState {
    creeper::ThemeManager& manager;
    pcs::Renderer& renderer;
};

auto VisualizationWindowComponent(VisualizationWindowState&) noexcept -> QPointer<QWidget>;
