#include "gui/working/open-control.hh"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <format>
#include <ranges>

#include <QStringList>

namespace pcs::gui::working {

namespace {

    auto normalized_extension(std::string extension) -> std::string {
        std::ranges::transform(extension, extension.begin(),
            [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        return extension;
    }

}

auto OpenControl::register_format(Format format) noexcept -> void {
    for (auto& extension : format.extensions) {
        extension = normalized_extension(extension);
    }
    formats.push_back(std::move(format));
}

auto OpenControl::dialog_filter() const noexcept -> QString {
    auto all_patterns = QStringList { };
    auto sections     = QStringList { };

    for (auto const& format : formats) {
        if (!format.dialog_pattern.empty()) {
            all_patterns.append(QString::fromStdString(format.dialog_pattern));
            sections.append(QString::fromStdString(
                std::format("{} ({})", format.label, format.dialog_pattern)));
        }
    }

    if (!all_patterns.isEmpty()) {
        sections.push_front(QString("资产文件 (%1)").arg(all_patterns.join(' ')));
    }

    return sections.join(";;");
}

auto OpenControl::open(std::string const& path) const noexcept -> std::expected<void, std::string> {
    const auto extension = normalized_extension(std::filesystem::path(path).extension().string());
    auto const* format   = find_format(extension);
    if (format == nullptr) {
        return std::unexpected { std::format("Unsupported asset file: {}", path) };
    }

    return format->open(path);
}

auto OpenControl::find_format(std::string_view extension) const noexcept -> Format const* {
    for (auto const& format : formats) {
        if (std::ranges::find(format.extensions, extension) != format.extensions.end()) {
            return &format;
        }
    }

    return nullptr;
}

}
