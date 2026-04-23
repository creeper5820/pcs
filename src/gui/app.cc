#include "app.hh"

#include "core/handle/points.hh"
#include "gui/context/features.hh"
#include "gui/context/modules.hh"
#include "gui/context/states.hh"
#include "gui/navigation.hh"
#include "gui/visualization-window.hh"
#include "gui/working-panel.hh"

#include <creeper-qt/core/application.hh>
#include <creeper-qt/layout/linear.hh>
#include <creeper-qt/utility/material-icon.hh>
#include <creeper-qt/utility/theme/preset/blue-miku.hh>
#include <creeper-qt/utility/theme/preset/green.hh>
#include <creeper-qt/utility/theme/theme.hh>
#include <creeper-qt/widget/cards/filled-card.hh>
#include <creeper-qt/widget/main-window.hh>

#include <spdlog/spdlog.h>

#include <qdir.h>
#include <qmessagebox.h>
#include <qshortcut.h>
#include <qstandardpaths.h>

#include <QKeySequence>

#include <utility>

using namespace pcs;
using namespace creeper;

namespace sp = spdlog;

struct App::Impl final {
public:
    explicit Impl(std::string const& configutation_location = "") noexcept {

        // use default user configuration location
        if (configutation_location.empty()) {
            const auto config_location =
                QStandardPaths::writableLocation(QStandardPaths::ConfigLocation);
            const auto full_location =
                config_location + QDir::separator() + application_name.c_str();

            configuration_path = full_location.toStdString();
        }

        // App modules loading
        {
            modules = gui::context::AppModules { };
            gui::context::register_default_features(modules);

            sp::info("App modules are loaded");
        }

        // App gui loading
        {
            manager = std::make_unique<ThemeManager>();
            manager->set_theme_pack(kBlueMikuThemePack);
            manager->set_color_mode(ColorMode::LIGHT);

            states = gui::context::AppStates { };

            window = MainWindowComponent();

            const auto& colorscheme = manager->theme_pack().dark;
            const auto& background  = colorscheme.background;
            modules.renderer->set_background(
                background.redF(), background.greenF(), background.blueF());

            const auto point = colorscheme.primary;
            // Set color for all pointcloud assets
            for (auto const& id : modules.assets->get_asset_ids()) {
                if (auto handle = modules.assets->get_handle<PointsHandle>(std::string { id })) {
                    auto* points = handle.value();
                    const auto alpha = std::get<3>(points->get_overall_color());
                    points->set_overall_color(point.redF(), point.greenF(), point.blueF(), alpha);
                }
            }

            modules.renderer->render_window();

            manager->apply_theme();

            // Q 键退出
            auto shortcut_q = new QShortcut { QKeySequence { Qt::Key_Q }, window };
            QObject::connect(shortcut_q, &QShortcut::activated, //
                [this] { exit_application_with_confirment(); });

            auto shortcut_undo = new QShortcut { QKeySequence::Undo, window };
            QObject::connect(
                shortcut_undo, &QShortcut::activated, [this] { modules.runtime->undo(); });

            auto shortcut_redo = new QShortcut { QKeySequence::Redo, window };
            QObject::connect(
                shortcut_redo, &QShortcut::activated, [this] { modules.runtime->redo(); });

            auto shortcut_redo_shift_z =
                new QShortcut { QKeySequence { Qt::CTRL | Qt::SHIFT | Qt::Key_Z }, window };
            QObject::connect(
                shortcut_redo_shift_z, &QShortcut::activated, [this] { modules.runtime->redo(); });

            auto shortcut_redo_y =
                new QShortcut { QKeySequence { Qt::CTRL | Qt::Key_Y }, window };
            QObject::connect(
                shortcut_redo_y, &QShortcut::activated, [this] { modules.runtime->redo(); });

            auto shortcut_save = new QShortcut { QKeySequence::Save, window };
            QObject::connect(shortcut_save, &QShortcut::activated, [this] {
                if (working_panel != nullptr) {
                    working_panel->save_current_asset();
                }
            });

            sp::info("App gui are loaded");
        }

        use_startup_files();

        sp::info("Applicatioin is loaded fully");
    }

    auto set_configuration_path(std::string const& path) noexcept {
        sp::info("The configuration path is modified to {}", path);
        configuration_path = path;
    }

    auto set_startup_files(std::vector<std::string> files) noexcept -> void {
        startup_files = std::move(files);
        use_startup_files();
    }

    auto set_theme_name(std::string const& name) noexcept -> void {
        if (name == "green") {
            manager->set_theme_pack(kGreenThemePack);
            manager->apply_theme();
            return;
        }
        if (name == "blue-miku" || name == "blue") {
            manager->set_theme_pack(kBlueMikuThemePack);
            manager->apply_theme();
            return;
        }

        sp::warn("Unknown theme '{}', keeping current theme", name);
    }

    auto request_exit() noexcept -> void { exit_requested = true; }

    auto should_exit() const noexcept -> bool { return exit_requested; }

    auto show() noexcept { window->show(); }

private:
    MainWindow* window = nullptr;

    std::string application_name = "pointcloud-shop";
    std::string configuration_path;
    std::vector<std::string> startup_files;
    bool exit_requested = false;

    WorkingPanel* working_panel = nullptr;
    gui::context::AppStates states;

    std::unique_ptr<ThemeManager> manager;
    gui::context::AppModules modules;

    auto MainWindowComponent() noexcept -> MainWindow* {
        namespace mwp = main_window::pro;
        namespace cp  = card::pro;
        namespace lp  = linear::pro;

        return new MainWindow {
            mwp::Central<FilledCard> {
                cp::ThemeManager { *manager },
                cp::MinimumSize { 1200, 800 },
                cp::Radius { 0 },
                cp::Layout<Row> {
                    lp::ContentsMargin { { 0, 0, 0, 0 } },
                    lp::Spacing { 0 },
                    lp::Item {
                        { 0 },
                        new Navigation {
                            *manager,
                            material::round::font,
                            std::vector<Navigation::ButtonContext> {
                                { "3d window", "home", [this] { } },
                                { "switch theme", "format_paint", [this] { switch_next_theme(); } },
                            },
                            [this](bool on) {
                                modules.mouse->set_mode(on ? gui::interaction::MouseModeId::Picker
                                                           : gui::interaction::MouseModeId::None);
                            },
                            [this] {
                                return modules.mouse->mode() == gui::interaction::MouseModeId::Picker;
                            },
                            [this] { exit_application_with_confirment(); },
                            *modules.mouse,
                        },
                    },
                    lp::Item {
                        { 255 },
                        new VisualizationWindow {
                            *manager,
                            *modules.renderer,
                            *modules.runtime,
                            *modules.mouse,
                        },
                    },
                    lp::Item {
                        { 0 },
                        [this]() {
                            working_panel = new WorkingPanel { *manager, *modules.assets,
                                *modules.runtime, *modules.renderer, *modules.open_control,
                                *modules.asset_details, *modules.mouse,
                                *modules.action_panels, states.working_panel_width,
                                states.assets_visibility };
                            return working_panel;
                        }(),
                    },
                },
            },
        };
    }

    auto use_configuration() noexcept { }

    auto use_startup_files() noexcept -> void {
        if (startup_files.empty() || working_panel == nullptr
            || modules.open_control == nullptr) {
            return;
        }

        auto last_opened_id = std::optional<std::string> { };
        for (auto const& file : startup_files) {
            if (auto result = modules.open_control->open(file); !result.has_value()) {
                sp::error("Failed to open startup file '{}': {}", file, result.error());
                continue;
            }

            working_panel->refresh_assets_list();
            auto id = modules.assets->last_asset_id();
            if (!id.empty()) {
                last_opened_id = id;
            }
        }

        if (last_opened_id.has_value()) {
            working_panel->select_asset(*last_opened_id);
        }

        startup_files.clear();
    }

    std::size_t current_theme_index = 0;
    auto switch_next_theme() noexcept -> void {
        constexpr auto array = std::array {
            kGreenThemePack,
            kBlueMikuThemePack,
        };

        if (current_theme_index == array.size()) //
            current_theme_index = 0;

        auto pack = array[current_theme_index];
        manager->set_theme_pack(pack);
        manager->apply_theme();

        current_theme_index += 1;
    }

    auto exit_application_with_confirment() noexcept -> void {
        auto confirmation_box = QMessageBox {
            QMessageBox::Question,
            "确认退出",
            "确认退出当前应用吗？",
            QMessageBox::Yes | QMessageBox::No,
            window,
        };
        confirmation_box.setDefaultButton(QMessageBox::No);

        auto result = confirmation_box.exec();
        if (result == QMessageBox::Yes) {
            QCoreApplication::quit();
        }
    }
};

App::App() noexcept
    : pimpl { std::make_unique<Impl>() } { }

App::~App() noexcept = default;

auto App::show() noexcept -> void { pimpl->show(); }

auto App::set_configuration_path(const std::string& path) noexcept -> void {
    pimpl->set_configuration_path(path);
}

auto App::set_startup_files(std::vector<std::string> files) noexcept -> void {
    pimpl->set_startup_files(std::move(files));
}

auto App::set_theme_name(std::string const& name) noexcept -> void { pimpl->set_theme_name(name); }

auto App::request_exit() noexcept -> void { pimpl->request_exit(); }

auto App::should_exit() const noexcept -> bool { return pimpl->should_exit(); }
