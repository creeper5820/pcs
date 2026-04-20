#include "file-option.hh"

namespace pcs {

FileOption::FileOption() noexcept
    : AppOption {
        QStringList {} << "f" << "file" << "filename",
        "The asset file to open",
        "file-path",
    } { }

auto FileOption::exec(Context& context) const -> std::expected<void, std::string> {
    if (!context.parser.isSet(*this)) {
        return { };
    }

    auto files = std::vector<std::string> { };
    files.push_back(context.parser.value(*this).toStdString());
    context.app.set_startup_files(std::move(files));
    return { };
}

}
