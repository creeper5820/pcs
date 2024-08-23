#include "workspace.hpp"

#include <memory>

#include "ui_workspace.h"

using namespace workspace;

struct Workspace::Impl {
 public:
  Impl() { ui_ = std::make_unique<Ui::Workspace>(); }

  ~Impl() {}

  std::unique_ptr<Ui::Workspace> ui_;
};

Workspace::Workspace() {
  pimpl_ = new Impl{};
  pimpl_->ui_->setupUi(this);
}

Workspace::~Workspace() { delete pimpl_; }