#include "qapplication.h"
#include <qurl.h>

#include "qwidget/workspace/viewer/viewer.hpp"

int main(int argc, char* argv[])
{
    auto app = QApplication { argc, argv };

    auto viewer = workspace::Viewer {};
    viewer.show();

    return app.exec();
}