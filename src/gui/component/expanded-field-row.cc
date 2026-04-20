#include "gui/component/expanded-field-row.hh"

#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/widget/text.hh>

#include <QSizePolicy>

#include <memory>

using namespace creeper;

namespace pcs::gui::component {

namespace {

    auto compact_measurements() noexcept -> OutlinedTextField::Measurements {
        auto measurements = OutlinedTextField::Measurements { };

        measurements.container_height     = 26;
        measurements.icon_rect_size       = 16;
        measurements.input_rect_size      = 16;
        measurements.label_rect_size      = 12;
        measurements.standard_font_height = 13;

        measurements.col_padding                      = 4;
        measurements.row_padding_without_icons        = 10;
        measurements.row_padding_with_icons           = 8;
        measurements.row_padding_populated_label_text = 0;
        measurements.padding_icons_text               = 8;

        measurements.supporting_text_and_character_counter_top_padding = 2;
        measurements.supporting_text_and_character_counter_row_padding = 8;

        return measurements;
    }

    auto make_expanded_field(theme::pro::ThemeManager const& theme, QFont const& font,
        QString const& default_value, QString const& placeholder) noexcept -> OutlinedTextField* {
        auto* field = new OutlinedTextField {
            theme,
            text_field::pro::Font { font },
            widget::pro::Apply { [](auto& self) {
                self.set_measurements(compact_measurements());
                self.setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
            } },
            common::pro::Text<text_field::pro::Token> { default_value },
        };

        if (!placeholder.isEmpty()) {
            field->setPlaceholderText(placeholder);
        }

        return field;
    }

    auto parse_double_input(OutlinedTextField& input, double fallback, double minimum,
        bool keep_empty_text) noexcept -> double {
        const auto raw_text = input.text().trimmed();
        if (keep_empty_text && raw_text.isEmpty()) {
            return fallback;
        }

        auto parsed = double { };
        auto ok     = false;
        parsed      = raw_text.toDouble(&ok);

        if (!ok || parsed < minimum) {
            parsed = fallback;
        }

        auto normalized = QString::number(parsed, 'f', 6);
        while (normalized.contains('.') && normalized.endsWith('0')) {
            normalized.chop(1);
        }
        if (normalized.endsWith('.')) {
            normalized.chop(1);
        }
        if (normalized == "-0") {
            normalized = "0";
        }

        input.setText(normalized);
        return parsed;
    }

}

struct ExpandedDualFieldRow::Impl {
    OutlinedTextField* first  = nullptr;
    OutlinedTextField* second = nullptr;
};

ExpandedDualFieldRow::ExpandedDualFieldRow(theme::pro::ThemeManager const& theme, QFont const& font,
    QString const& first_default, QString const& second_default,
    QString const& first_placeholder, QString const& second_placeholder) noexcept
    : pimpl { std::make_unique<Impl>() } {
    pimpl->first  = make_expanded_field(theme, font, first_default, first_placeholder);
    pimpl->second = make_expanded_field(theme, font, second_default, second_placeholder);

    auto* row = new Row {
        row::pro::Spacing { 6 },
        row::pro::Item { { 1, Qt::AlignVCenter }, pimpl->first },
        row::pro::Item { { 1, Qt::AlignVCenter }, pimpl->second },
    };
    row->setContentsMargins(0, 0, 0, 0);
    setLayout(row);
}

ExpandedDualFieldRow::~ExpandedDualFieldRow() noexcept = default;

auto ExpandedDualFieldRow::first() const noexcept -> OutlinedTextField& { return *pimpl->first; }

auto ExpandedDualFieldRow::second() const noexcept -> OutlinedTextField& { return *pimpl->second; }

auto ExpandedDualFieldRow::parse_first_double(double fallback, double minimum) noexcept -> double {
    return parse_double_input(*pimpl->first, fallback, minimum, false);
}

auto ExpandedDualFieldRow::parse_second_double(double fallback, double minimum) noexcept -> double {
    return parse_double_input(*pimpl->second, fallback, minimum, false);
}

auto ExpandedDualFieldRow::parse_first_double_keep_empty(double fallback, double minimum) noexcept
    -> double {
    return parse_double_input(*pimpl->first, fallback, minimum, true);
}

auto ExpandedDualFieldRow::parse_second_double_keep_empty(double fallback, double minimum) noexcept
    -> double {
    return parse_double_input(*pimpl->second, fallback, minimum, true);
}

struct ExpandedTripleFieldRow::Impl {
    OutlinedTextField* first  = nullptr;
    OutlinedTextField* second = nullptr;
    OutlinedTextField* third  = nullptr;
};

ExpandedTripleFieldRow::ExpandedTripleFieldRow(theme::pro::ThemeManager const& theme,
    QFont const& font, QString const& first_default, QString const& second_default,
    QString const& third_default, QString const& first_placeholder,
    QString const& second_placeholder, QString const& third_placeholder) noexcept
    : pimpl { std::make_unique<Impl>() } {
    pimpl->first  = make_expanded_field(theme, font, first_default, first_placeholder);
    pimpl->second = make_expanded_field(theme, font, second_default, second_placeholder);
    pimpl->third  = make_expanded_field(theme, font, third_default, third_placeholder);

    auto* row = new Row {
        row::pro::Spacing { 6 },
        row::pro::Item { { 1, Qt::AlignVCenter }, pimpl->first },
        row::pro::Item { { 1, Qt::AlignVCenter }, pimpl->second },
        row::pro::Item { { 1, Qt::AlignVCenter }, pimpl->third },
    };
    row->setContentsMargins(0, 0, 0, 0);
    setLayout(row);
}

ExpandedTripleFieldRow::~ExpandedTripleFieldRow() noexcept = default;

auto ExpandedTripleFieldRow::first() const noexcept -> OutlinedTextField& { return *pimpl->first; }

auto ExpandedTripleFieldRow::second() const noexcept -> OutlinedTextField& { return *pimpl->second; }

auto ExpandedTripleFieldRow::third() const noexcept -> OutlinedTextField& { return *pimpl->third; }

auto ExpandedTripleFieldRow::parse_first_double(double fallback, double minimum) noexcept -> double {
    return parse_double_input(*pimpl->first, fallback, minimum, false);
}

auto ExpandedTripleFieldRow::parse_second_double(double fallback, double minimum) noexcept -> double {
    return parse_double_input(*pimpl->second, fallback, minimum, false);
}

auto ExpandedTripleFieldRow::parse_third_double(double fallback, double minimum) noexcept -> double {
    return parse_double_input(*pimpl->third, fallback, minimum, false);
}

auto ExpandedTripleFieldRow::parse_first_double_keep_empty(double fallback, double minimum) noexcept
    -> double {
    return parse_double_input(*pimpl->first, fallback, minimum, true);
}

auto ExpandedTripleFieldRow::parse_second_double_keep_empty(double fallback, double minimum) noexcept
    -> double {
    return parse_double_input(*pimpl->second, fallback, minimum, true);
}

auto ExpandedTripleFieldRow::parse_third_double_keep_empty(double fallback, double minimum) noexcept
    -> double {
    return parse_double_input(*pimpl->third, fallback, minimum, true);
}

struct AxisRangeRow::Impl {
    Text* label              = nullptr;
    OutlinedTextField* first = nullptr;
    OutlinedTextField* second = nullptr;
};

AxisRangeRow::AxisRangeRow(theme::pro::ThemeManager const& theme, QFont const& font,
    QString const& axis_label, QString const& first_default, QString const& second_default,
    QString const& first_placeholder, QString const& second_placeholder) noexcept
    : pimpl { std::make_unique<Impl>() } {
    auto label_font = font;
    label_font.setBold(true);

    pimpl->label = new Text {
        theme,
        text::pro::Font { label_font },
        text::pro::Text { axis_label },
        text::pro::Alignment { Qt::AlignVCenter | Qt::AlignLeft },
    };
    pimpl->label->setMinimumWidth(16);
    pimpl->label->setMaximumWidth(20);

    pimpl->first  = make_expanded_field(theme, font, first_default, first_placeholder);
    pimpl->second = make_expanded_field(theme, font, second_default, second_placeholder);

    auto* row = new Row {
        row::pro::Spacing { 8 },
        row::pro::Item { { 1, Qt::AlignVCenter }, pimpl->label },
        row::pro::Item { { 7, Qt::AlignVCenter }, pimpl->first },
        row::pro::Item { { 7, Qt::AlignVCenter }, pimpl->second },
    };
    row->setContentsMargins(0, 1, 0, 1);
    setLayout(row);
}

AxisRangeRow::~AxisRangeRow() noexcept = default;

auto AxisRangeRow::first() const noexcept -> OutlinedTextField& { return *pimpl->first; }

auto AxisRangeRow::second() const noexcept -> OutlinedTextField& { return *pimpl->second; }

auto AxisRangeRow::parse_first_double(double fallback, double minimum) noexcept -> double {
    return parse_double_input(*pimpl->first, fallback, minimum, false);
}

auto AxisRangeRow::parse_second_double(double fallback, double minimum) noexcept -> double {
    return parse_double_input(*pimpl->second, fallback, minimum, false);
}

auto AxisRangeRow::parse_first_double_keep_empty(double fallback, double minimum) noexcept
    -> double {
    return parse_double_input(*pimpl->first, fallback, minimum, true);
}

auto AxisRangeRow::parse_second_double_keep_empty(double fallback, double minimum) noexcept
    -> double {
    return parse_double_input(*pimpl->second, fallback, minimum, true);
}

}
