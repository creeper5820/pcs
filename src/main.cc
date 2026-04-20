#include "app/options/app-options.hh"
#include "app/options/config-option.hh"
#include "app/options/egg-option.hh"
#include "app/options/file-option.hh"
#include "app/options/theme-option.hh"
#include "gui/app.hh"

#include <creeper-qt/core/application.hh>
#include <spdlog/spdlog.h>

using namespace creeper;

auto main(int argc, char* argv[]) -> int {
    app::init {
        // app::pro::Attribute { Qt::AA_EnableHighDpiScaling },
        // app::pro::Attribute { Qt::AA_UseHighDpiPixmaps },
        app::pro::Complete { argc, argv },
    };

    auto options = pcs::AppOptions {
        std::make_unique<pcs::EggOption>(),
        std::make_unique<pcs::FileOption>(),
        std::make_unique<pcs::ConfigOption>(),
        std::make_unique<pcs::ThemeOption>(),
        std::make_unique<pcs::ListThemesOption>(),
    };
    options.process(*qApp);

    auto application = pcs::App { };
    if (auto result = options.exec(application); !result.has_value()) {
        spdlog::error("{}", result.error());
        return 1;
    }

    if (application.should_exit()) {
        return 0;
    }

    application.show();

    return app::exec();
}
