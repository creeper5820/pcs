#pragma once

#include "gui/app.hh"

#include <expected>
#include <memory>
#include <string>

#include <qcommandlineoption.h>
#include <qcommandlineparser.h>

namespace pcs {

class AppOption : public QCommandLineOption {
public:
    struct Context {
        QCommandLineParser const& parser;
        App& app;
    };

    using QCommandLineOption::QCommandLineOption;

    virtual ~AppOption() = default;

    virtual auto exec(Context& context) const -> std::expected<void, std::string> = 0;
};

}
