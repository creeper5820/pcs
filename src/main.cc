#include "gui/app.hh"

#include <creeper-qt/core/application.hh>
#include <qcommandlineparser.h>
#include <spdlog/spdlog.h>

using namespace creeper;

auto main(int argc, char* argv[]) -> int {
    app::init {
        // app::pro::Attribute { Qt::AA_EnableHighDpiScaling },
        // app::pro::Attribute { Qt::AA_UseHighDpiPixmaps },
        app::pro::Complete { argc, argv },
    };

    auto parser = QCommandLineParser { };
    parser.setApplicationDescription(parser.tr("Point Cloud Shop With PCL Backend"));

    parser.addVersionOption();
    parser.addHelpOption();

    {
        auto option = QCommandLineOption {
            "show-egg",
            "Show a egg tip",
        };
        parser.addOption(option);
    }
    {
        auto option = QCommandLineOption {
            QStringList { } << "f" << "file",
            "The point cloud file to open",
            "file-path",
        };
        parser.addOption(option);
    }
    {
        auto option = QCommandLineOption {
            QStringList { } << "c" << "config",
            "The path of config file",
            "config-path",
        };
        parser.addOption(option);
    }
    {
        auto option = QCommandLineOption {
            "theme",
            "Set the application's color theme, e.g., 'green' or 'dark'.",
            "theme-name",
        };
        parser.addOption(option);
    }
    {
        auto option = QCommandLineOption {
            "list-themes",
            "List all available theme colors.",
        };
        parser.addOption(option);
    }
    parser.process(*qApp);

    auto application = pcs::App { };
    if (parser.isSet("show-egg")) {
        spdlog::info("Egg triggered! 🥚");
        return 0;
    }
    if (parser.isSet("list-themes")) {
        return 0;
    }
    if (parser.isSet("c")) {
        auto path = parser.value("config-path");
        application.set_configuration_path(path.toStdString());
    }
    // ...

    application.show();

    return app::exec();
}
