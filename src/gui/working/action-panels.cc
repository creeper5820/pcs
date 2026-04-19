#include "gui/working/action-panels.hh"

#include <creeper-qt/widget/text.hh>

#include <QSizePolicy>

#include <algorithm>

namespace pcs::gui::working {

auto ActionPanelRegistry::register_factory(pcs::AssetKind kind, Factory factory) noexcept -> void {
    factories[kind] = std::move(factory);
}

auto ActionPanelRegistry::create(pcs::AssetKind kind, ActionPanelContext context,
    QFont const& font) const noexcept -> std::unique_ptr<AssetActionPanel> {
    auto iter = factories.find(kind);
    if (iter == factories.end() || !iter->second) {
        return nullptr;
    }

    return iter->second(std::move(context), font);
}

ActionPanelHost::ActionPanelHost(
    ActionPanelContext context, ActionPanelRegistry const* registry, QFont const& font)
    : context { std::move(context) }
    , registry { registry }
    , font { font } {
    placeholder = new Text {
        theme::pro::ThemeManager { *this->context.manager },
        text::pro::Text { "暂无专属操作" },
        text::pro::Alignment { Qt::AlignHCenter },
        text::pro::Font { this->font },
    };

    stack = new Stacked {
        stacked::pro::Item { placeholder },
        stacked::pro::CurrentIndex { kPlaceholderIndex },
    };

    root = new Widget {
        widget::pro::Apply {
            [](QWidget& self) { self.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed); } },
        widget::pro::Layout { stack },
    };

    sync_current_panel_height();
}

auto ActionPanelHost::widget() const noexcept -> QWidget* { return root; }

auto ActionPanelHost::clear() noexcept -> void {
    for (auto& [_, panel] : panels) {
        if (panel) {
            panel->clear();
        }
    }

    stack->setCurrentIndex(kPlaceholderIndex);
    sync_current_panel_height();
}

auto ActionPanelHost::bind_asset(pcs::AssetKind kind, std::string const& id) noexcept -> void {
    auto* panel = ensure_panel(kind);
    if (panel == nullptr) {
        stack->setCurrentIndex(kPlaceholderIndex);
        sync_current_panel_height();
        return;
    }

    panel->bind_asset(id);
    stack->setCurrentIndex(panel_indices[kind]);
    sync_current_panel_height();
}

auto ActionPanelHost::ensure_panel(pcs::AssetKind kind) noexcept -> AssetActionPanel* {
    if (auto iter = panels.find(kind); iter != panels.end() && iter->second != nullptr) {
        return iter->second.get();
    }

    if (registry == nullptr) {
        return nullptr;
    }

    auto panel = registry->create(kind, context, font);
    if (panel == nullptr || panel->widget() == nullptr) {
        return nullptr;
    }

    const auto index    = stack->addWidget(panel->widget());
    panel_indices[kind] = index;

    if (context.manager != nullptr) {
        context.manager->apply_theme();
    }

    auto* raw    = panel.get();
    panels[kind] = std::move(panel);
    return raw;
}

auto ActionPanelHost::sync_current_panel_height() noexcept -> void {
    if (root == nullptr || stack == nullptr) {
        return;
    }

    auto* current = stack->currentWidget();
    if (current == nullptr) {
        return;
    }

    if (auto* layout = current->layout()) {
        layout->activate();
    }

    const auto height = std::max(current->minimumSizeHint().height(), current->sizeHint().height());
    root->setFixedHeight(height);
    root->updateGeometry();
}

}
