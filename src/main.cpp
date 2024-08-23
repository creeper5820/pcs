#include "qapplication.h"
#include <QtQml/qqmlapplicationengine.h>

// #include "ui/workspace/viewer/viewer.hpp"
// #include "ui/workspace/workspace.hpp"

int main(int argc, char* argv[])
{
    auto app = QApplication { argc, argv };
    auto qml = QQmlApplicationEngine {};
    qml.load(QUrl(QStringLiteral("qrc:/viewer.qml")));

    // auto a = workspace::Viewer {};
    // a.show();

    return app.exec();
}