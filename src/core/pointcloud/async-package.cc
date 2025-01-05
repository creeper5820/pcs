#include "async-package.hh"
#include "core/renderer/renderer.hh"
#include "core/share/cloud-box.hh"
#include "core/share/object.hh"

#include <future>

using namespace std::chrono_literals;
using namespace core::cloud;

struct CloudPackage::Impl {
    std::future<std::unique_ptr<CloudSource>> source;
    std::unique_ptr<CloudObject> object;
};

CloudPackage::CloudPackage(const std::string& path)
    : pimpl_(new Impl) {
    pimpl_->source
        = std::async(std::launch::async, [path] { return std::make_unique<CloudSource>(path); });
}
CloudPackage::~CloudPackage() { delete pimpl_; }

bool CloudPackage::tryToLoadRenderer() {
    if (pimpl_->source.wait_for(100ms) != std::future_status::ready) return false;
    pimpl_->object = Renderer::instance().makeCloud(*pimpl_->source.get());
    return true;
}