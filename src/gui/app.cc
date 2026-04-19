#include "app.hh"

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
            modules = gui::context::make_app_modules();
            gui::context::register_default_features(modules);

            sp::info("App modules are loaded");
        }

        // App gui loading
        {
            manager = std::make_unique<ThemeManager>();
            manager->set_theme_pack(kBlueMikuThemePack);
            manager->set_color_mode(ColorMode::LIGHT);

            states = gui::context::make_app_states(*manager, modules);

            {
                navigation_state = std::move(states.navigation);

                navigation_state->icon_font     = material::round::font;
                navigation_state->mouse         = modules.mouse.get();
                navigation_state->function_quit = [this] { exit_application_with_confirment(); };
                navigation_state->picker_mode_getter = [this] {
                    return modules.mouse->mode() == gui::interaction::MouseModeId::Picker;
                };
                navigation_state->picker_mode_setter = [this](bool on) {
                    modules.mouse->set_mode(on ? gui::interaction::MouseModeId::Picker
                                               : gui::interaction::MouseModeId::None);
                };

                auto& contexts = navigation_state->buttons_context;
                contexts.emplace_back("3d window", "home", [this] { });
                contexts.emplace_back(
                    "switch theme", "format_paint", [this] { switch_next_theme(); });
            }
            { visualization_window_state = std::move(states.visualization); }
            { working_panel_state = std::move(states.working); }

            window = MainWindowComponent();

            const auto& colorscheme = manager->theme_pack().dark;
            const auto& background  = colorscheme.background;
            modules.renderer->set_background(
                background.redF(), background.greenF(), background.blueF());

            const auto point = colorscheme.primary;
            modules.assets->set_default_point_color(point.redF(), point.greenF(), point.blueF());

            modules.renderer->render_window();

            manager->apply_theme();

            // Q 键退出
            auto shortcut_q = new QShortcut { Qt::Key_Q, window };
            QObject::connect(shortcut_q, &QShortcut::activated, //
                [this] { exit_application_with_confirment(); });

            // C 键居中
            auto shortcut_c = new QShortcut { Qt::Key_C, window };
            QObject::connect(shortcut_c, &QShortcut::activated,
                [this] { window->apply(widget::pro::MoveCenter { }); });

            sp::info("App gui are loaded");
        }

        sp::info("Applicatioin is loaded fully");
    }

    auto set_configuration_path(std::string const& path) noexcept {
        sp::info("The configuration path is modified to {}", path);
        configuration_path = path;
    }

    auto show() noexcept { window->show(); }

private:
    QPointer<MainWindow> window;

    std::string application_name = "pointcloud-shop";
    std::string configuration_path;

    std::unique_ptr<NavigationState> navigation_state;
    std::unique_ptr<VisualizationWindowState> visualization_window_state;
    std::unique_ptr<WorkingPanelState> working_panel_state;
    gui::context::AppStates states;

    std::unique_ptr<ThemeManager> manager;
    gui::context::AppModules modules;

    auto MainWindowComponent() noexcept -> QPointer<MainWindow> {
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
                        NavigationComponent(*navigation_state).get(),
                    },
                    lp::Item {
                        { 255 },
                        VisualizationWindowComponent(*visualization_window_state).get(),
                    },
                    lp::Item {
                        { 0 },
                        WorkingPanelComponent(*working_panel_state).get(),
                    },
                },
            },
        };
    }

    auto use_configuration() noexcept { }

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
