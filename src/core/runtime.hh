#pragma once

#include "utility/pimpl.hh"

#include <future>
#include <concepts>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>

namespace pcs {

namespace runtime::internal {

    struct MetaView {
        std::string name;
        bool recordable = false;
        bool redoable   = true;
        bool main_thread = false;
    };

    struct IEvent {
        MetaView meta { };
        bool has_executed = false;

        virtual ~IEvent() = default;

        virtual auto exec() noexcept -> void = 0;
        virtual auto undo() noexcept -> void = 0;
        virtual auto redo() noexcept -> void = 0;

        auto replay() noexcept -> void {
            if (has_executed) {
                redo();
                return;
            }

            exec();
        }
    };

}

class Runtime {
    PCS_PIMPL_DEFINITION(Runtime)

public:
    auto set_operation_sink(std::function<void(std::string const&)> sink) noexcept -> void;

    auto undo() noexcept -> std::future<bool>;
    auto redo() noexcept -> std::future<bool>;

    auto can_undo() const noexcept -> bool;
    auto can_redo() const noexcept -> bool;

    template <class Event>
    auto submit(Event event) noexcept {
        static_assert(requires(Event e) {
            { e.meta.name } -> std::convertible_to<std::string_view>;
            { e.meta.recordable } -> std::convertible_to<bool>;
            { e.meta.redoable } -> std::convertible_to<bool>;
            { e.exec() };
            { e.redo() };
        }, "Event must provide meta{name, recordable, redoable}, exec() and redo()");

        using R = decltype(event.exec());
        struct Task final : runtime::internal::IEvent {
            std::promise<R> promise;
            Event inner;

            explicit Task(Event e) noexcept
                : inner { std::move(e) } {
                this->meta.name       = std::string { std::string_view { inner.meta.name } };
                this->meta.recordable = inner.meta.recordable;
                this->meta.redoable   = inner.meta.redoable;
                if constexpr (requires { inner.meta.main_thread; }) {
                    this->meta.main_thread = static_cast<bool>(inner.meta.main_thread);
                }
            }

            auto exec() noexcept -> void override {
                if constexpr (std::is_void_v<R>) {
                    inner.exec();
                    promise.set_value();
                } else {
                    promise.set_value(inner.exec());
                }
                this->has_executed = true;
            }

            auto undo() noexcept -> void override {
                if constexpr (requires { inner.undo(); }) {
                    if constexpr (std::is_void_v<decltype(inner.undo())>) {
                        inner.undo();
                    } else {
                        (void)inner.undo();
                    }
                }
            }

            auto redo() noexcept -> void override {
                if constexpr (std::is_void_v<R>) {
                    inner.redo();
                } else {
                    (void)inner.redo();
                }
                this->has_executed = true;
            }
        };

        auto task   = std::make_unique<Task>(std::move(event));
        auto future = task->promise.get_future();
        submit_internal(std::unique_ptr<runtime::internal::IEvent>(std::move(task)));
        return future;
    }

private:
    auto submit_internal(std::unique_ptr<runtime::internal::IEvent> event) noexcept -> void;
};

}
