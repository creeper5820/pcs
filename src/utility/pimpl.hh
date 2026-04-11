#pragma once
#include <memory>

#define PCS_PIMPL_DEFINITION(Class)                                                                \
public:                                                                                            \
    explicit Class() noexcept;                                                                     \
    ~Class() noexcept;                                                                             \
    Class(const Class&)            = delete;                                                       \
    Class& operator=(const Class&) = delete;                                                       \
                                                                                                   \
private:                                                                                           \
    struct Impl;                                                                                   \
    std::unique_ptr<Impl> pimpl;
