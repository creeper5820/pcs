#include "qapplication.h"

#include "qt/workspace/main-window.hh"

int main(int argc, char* argv[]) {
    creeper::Theme::setTheme("common-green");
    auto app = QApplication { argc, argv };

    auto window = qt::Workspace {};
    window.show();

    return app.exec();
}