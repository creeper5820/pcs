#pragma once

#include <string>

namespace core::cloud {

class CloudPackage {
public:
    explicit CloudPackage(const std::string& path);
    ~CloudPackage();
    CloudPackage(const CloudPackage&) = delete;
    CloudPackage& operator=(const CloudPackage&) = delete;

    /// @brief read file when this package is created.
    /// @return if the future source is loaded, return true.
    /// @note before return, the object will be loaded to renderer.
    bool tryToLoadRenderer();

private:
    struct Impl;
    Impl* pimpl_;
};

}