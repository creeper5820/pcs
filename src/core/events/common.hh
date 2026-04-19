#pragma once
#include <coroutine>
#include <future>
#include <string_view>

namespace pcs::event {

struct EventMeta {
    std::string_view name = "Unknown Name";
    bool consuming        = false;
};
struct EventTask {
    using Meta = EventMeta;

    virtual ~EventTask() = default;

    virtual auto meta() const noexcept -> const Meta& = 0;
    virtual auto exec() noexcept -> void { }
};

}
namespace pcs::co {

struct runtime_executor {
    struct task {
        virtual ~task() = default;
        virtual auto run() noexcept -> void { }
    };

    virtual ~runtime_executor() = default;
    virtual auto submit_task(std::unique_ptr<task>) noexcept -> void { }

    template <typename F>
    auto submit_task(F&& task) noexcept -> void
        requires std::invocable<F>
    {
        struct Instantiated : task {
            std::decay_t<F> f;
            Instantiated(F&& f)
                : f { std::forward<F>(f) } { }
            ~Instantiated() = default;
            auto run() noexcept -> void override { f(); }
        };
        submit_task(std::make_unique<Instantiated>(std::forward<F>(task)));
    }
};

/// @note
/// 为了和 promise_type 的命名风格对齐，和协程相关的类型就用小写吧
template <typename result_t>
struct runtime_task {

    struct promise_type {
        std::promise<result_t> promise;

        auto get_return_object() {
            return runtime_task {
                std::coroutine_handle<promise_type>::from_promise(*this),
            };
        }

        constexpr auto initial_suspend() const noexcept { return std::suspend_always { }; }

        constexpr auto final_suspend() const noexcept { return std::suspend_always { }; }

        auto return_value(result_t t) { promise.set_value(std::move(t)); }

        auto unhandled_exception() { promise.set_exception(std::current_exception()); }
    };

    std::coroutine_handle<promise_type> handle;

    auto get_future() -> std::future<result_t> {
        auto& co_promise = handle.promise();
        return co_promise.promise.get_future();
    }

    ~runtime_task() noexcept {
        if (handle) handle.destroy();
    }
};

struct switch_to {

    runtime_executor& exetutor;

    explicit switch_to(runtime_executor& exetutor)
        : exetutor { exetutor } { }

    // Always stop to switch exetutor on await_suspend
    constexpr auto await_ready() const noexcept { return false; }

    auto await_suspend(std::coroutine_handle<> co) const noexcept {
        exetutor.submit_task([co] {
            // Resume the coroutine context in the exetutor env
            co.resume();
        });
    }

    constexpr auto await_resume() const noexcept { }
};

}
