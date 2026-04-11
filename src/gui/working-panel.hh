#pragma once
#include "core/assets.hh"

#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/utility/wrapper/mutable-value.hh>
#include <creeper-qt/utility/wrapper/widget.hh>

#include <qpointer.h>
#include <qwidget.h>

struct WorkingPanelState {
    creeper::ThemeManager& manager;
    pcs::AssetsManager& assets;
    pcs::Renderer& renderer;

    creeper::MutableDouble panel_width { 300 };

    bool pointcloud_visibility = true;
};
auto WorkingPanelComponent(WorkingPanelState&) noexcept -> QPointer<QWidget>;
