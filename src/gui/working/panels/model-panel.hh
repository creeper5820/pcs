#pragma once

#include "gui/working/action-panels.hh"
#include "utility/pimpl.hh"

namespace pcs::gui::working {

class ModelPanel final : public AssetActionPanel {
    PCS_PIMPL_DEFINITION(ModelPanel)

public:
    explicit ModelPanel(ActionPanelContext context, QFont const& font) noexcept;

    auto widget() const noexcept -> QWidget* override;
    auto bind_asset(std::string const& id) noexcept -> void override;
    auto clear() noexcept -> void override;
};

}
