#include <QApplication>

#include "ui/workspace/viewer/viewer.hpp"
#include "ui/workspace/workspace.hpp"

int main(int argc, char* argv[]) {
  auto app = QApplication{argc, argv};

  // auto a = workspace::Workspace {};
  auto a = workspace::Viewer{};
  a.show();

  return app.exec();
}