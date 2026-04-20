#pragma once

#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/utility/wrapper/pimpl.hh>
#include <creeper-qt/widget/text-fields.hh>

#include <QWidget>

namespace pcs::gui::component {

class ExpandedDualFieldRow final : public QWidget {
    CREEPER_PIMPL_DEFINITION(ExpandedDualFieldRow)

public:
    ExpandedDualFieldRow(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
        QString const& first_default, QString const& second_default,
        QString const& first_placeholder = {}, QString const& second_placeholder = {}) noexcept;

    auto first() const noexcept -> creeper::OutlinedTextField&;
    auto second() const noexcept -> creeper::OutlinedTextField&;

    auto parse_first_double(double fallback, double minimum) noexcept -> double;
    auto parse_second_double(double fallback, double minimum) noexcept -> double;
    auto parse_first_double_keep_empty(double fallback, double minimum) noexcept -> double;
    auto parse_second_double_keep_empty(double fallback, double minimum) noexcept -> double;
};

class AxisRangeRow final : public QWidget {
    CREEPER_PIMPL_DEFINITION(AxisRangeRow)

public:
    AxisRangeRow(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
        QString const& axis_label, QString const& first_default, QString const& second_default,
        QString const& first_placeholder = {}, QString const& second_placeholder = {}) noexcept;

    auto first() const noexcept -> creeper::OutlinedTextField&;
    auto second() const noexcept -> creeper::OutlinedTextField&;

    auto parse_first_double(double fallback, double minimum) noexcept -> double;
    auto parse_second_double(double fallback, double minimum) noexcept -> double;
    auto parse_first_double_keep_empty(double fallback, double minimum) noexcept -> double;
    auto parse_second_double_keep_empty(double fallback, double minimum) noexcept -> double;
};

class ExpandedTripleFieldRow final : public QWidget {
    CREEPER_PIMPL_DEFINITION(ExpandedTripleFieldRow)

public:
    ExpandedTripleFieldRow(creeper::theme::pro::ThemeManager const& theme, QFont const& font,
        QString const& first_default, QString const& second_default, QString const& third_default,
        QString const& first_placeholder = {}, QString const& second_placeholder = {},
        QString const& third_placeholder = {}) noexcept;

    auto first() const noexcept -> creeper::OutlinedTextField&;
    auto second() const noexcept -> creeper::OutlinedTextField&;
    auto third() const noexcept -> creeper::OutlinedTextField&;

    auto parse_first_double(double fallback, double minimum) noexcept -> double;
    auto parse_second_double(double fallback, double minimum) noexcept -> double;
    auto parse_third_double(double fallback, double minimum) noexcept -> double;
    auto parse_first_double_keep_empty(double fallback, double minimum) noexcept -> double;
    auto parse_second_double_keep_empty(double fallback, double minimum) noexcept -> double;
    auto parse_third_double_keep_empty(double fallback, double minimum) noexcept -> double;
};

}
