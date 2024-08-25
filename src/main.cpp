#include "qapplication.h"
#include "qwidget/workspace/workspace.hpp"

int main(int argc, char* argv[]) {
    auto app = QApplication { argc, argv };

    auto workspace = workspace::Workspace {};
    workspace.show();

    return app.exec();
}