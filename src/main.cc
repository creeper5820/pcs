#include "qapplication.h"
#include "widget/workspace/workspace.hh"

int main(int argc, char* argv[]) {
    using namespace workspace;

    auto app = QApplication { argc, argv };

    auto w = Workspace {};
    w.show();

    return app.exec();
}