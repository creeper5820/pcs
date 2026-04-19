#pragma once
#include "core/events/common.hh"
#include "utility/pimpl.hh"

namespace pcs {

class Runtime {
    PCS_PIMPL_DEFINITION(Runtime)

public:
    using Task = event::EventTask;

    auto submit(std::unique_ptr<Task>) noexcept -> void;

    template <class Event>
    auto submit(std::unique_ptr<typename Event::Context> context) noexcept {

        constexpr auto has_context = requires {
            typename Event::Context;
            typename Event::Result;
        };
        static_assert(has_context, "Event must has context and result");

        using R = typename Event::Result;
        using C = typename Event::Context;

        const auto& meta = context->meta;
        // Do something for record

        auto promise = std::promise<R> { };
        auto future  = promise.get_future();

        struct Instantiated : public Task {
            std::promise<R> promise;
            std::unique_ptr<C> context;

            explicit Instantiated(std::promise<R> p, std::unique_ptr<C> c) noexcept
                : promise { std::move(p) }
                , context { std::move(c) } { }

            auto meta() const noexcept -> const Task::Meta& override {
                constexpr auto has_meta = requires(C c) {
                    {
                        auto { c.meta }
                    } -> std::same_as<Task::Meta>;
                };
                static_assert(has_meta, "Event Context must has meta");
                return context->meta;
            }
            auto exec() noexcept -> void override {
                constexpr auto has_runtime_exec =
                    requires { Event::runtime_exec(std::declval<std::unique_ptr<C>>()); };
                static_assert(has_runtime_exec, "Event must has runtime_exec");

                auto result = Event::runtime_exec(std::move(context));
                promise.set_value(std::move(result));
            }
        };

        auto task = std::make_unique<Instantiated>(std::move(promise), std::move(context));

        // Push to task queue
        submit(std::move(task));

        return future;
    }
};

}
