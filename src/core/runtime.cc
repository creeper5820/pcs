#include "runtime.hh"

#include <qcoreapplication.h>
#include <qmetaobject.h>
#include <qrunnable.h>
#include <qthread.h>
#include <qthreadpool.h>

#include <spdlog/spdlog.h>

#include <functional>
#include <mutex>
#include <type_traits>
#include <utility>
#include <vector>

namespace pcs {

struct Runtime::Impl {
    struct TaskBase {
        virtual ~TaskBase() = default;
        virtual auto run() noexcept -> void = 0;
    };

    template <typename F>
    struct Task final : TaskBase {
        explicit Task(F&& f) noexcept
            : fn { std::forward<F>(f) } { }

        auto run() noexcept -> void override { fn(); }

        F fn;
    };

    struct HistoryRecord {
        std::unique_ptr<runtime::internal::IEvent> event;
    };

    mutable std::mutex history_guard;
    std::vector<HistoryRecord> history;
    std::size_t cursor = 0;

    mutable std::mutex sink_guard;
    std::function<void(std::string const&)> operation_sink;

    explicit Impl() = default;

    auto in_main_thread() const noexcept -> bool {
        auto* app = QCoreApplication::instance();
        if (app == nullptr) {
            return false;
        }

        return QThread::currentThread() == app->thread();
    }

    auto submit_on_main(std::unique_ptr<TaskBase> task) noexcept -> void {
        QMetaObject::invokeMethod(
            QCoreApplication::instance(),
            [task = std::move(task)]() mutable {
                if (task != nullptr) {
                    task->run();
                }
            },
            Qt::QueuedConnection);
    }

    auto submit_on_work(std::unique_ptr<TaskBase> task) noexcept -> void {
        struct TaskRunnable final : QRunnable {
            explicit TaskRunnable(std::unique_ptr<TaskBase> t) noexcept
                : task { std::move(t) } {
                setAutoDelete(true);
            }

            auto run() -> void override {
                if (task != nullptr) {
                    task->run();
                }
            }

            std::unique_ptr<TaskBase> task;
        };

        QThreadPool::globalInstance()->start(new TaskRunnable { std::move(task) });
    }

    auto emit_operation(std::string message) noexcept -> void {
        std::function<void(std::string const&)> sink;
        {
            auto lock = std::scoped_lock { sink_guard };
            sink      = operation_sink;
        }

        if (!sink) {
            return;
        }

        QMetaObject::invokeMethod(QCoreApplication::instance(),
            [sink = std::move(sink), message = std::move(message)]() { sink(message); },
            Qt::QueuedConnection);
    }

    template <typename F>
    auto submit_on_executor(bool main_thread, F&& f) noexcept -> void {
        auto task = std::make_unique<Task<std::decay_t<F>>>(std::forward<F>(f));
        if (main_thread) {
            if (in_main_thread()) {
                task->run();
                return;
            }

            submit_on_main(std::move(task));
            return;
        }

        submit_on_work(std::move(task));
    }

    auto push_history(std::unique_ptr<runtime::internal::IEvent> event) noexcept -> void {
        if (event == nullptr || !event->meta.recordable) {
            return;
        }

        auto lock = std::scoped_lock { history_guard };

        if (cursor < history.size()) {
            history.erase(history.begin() + static_cast<std::ptrdiff_t>(cursor), history.end());
        }

        history.push_back(HistoryRecord { std::move(event) });
        cursor = history.size();
    }

    auto submit_task(std::unique_ptr<runtime::internal::IEvent> task) noexcept {
        const auto run_on_main = task != nullptr && task->meta.main_thread;
        submit_on_executor(run_on_main,
            [this, task = std::move(task)]() mutable {
                if (task == nullptr) {
                    return;
                }

                spdlog::info("[runtime] exec: {}", task->meta.name);
                emit_operation(std::string { "执行: " } + task->meta.name);
                task->exec();
                spdlog::info("[runtime] done: {}", task->meta.name);
                emit_operation(std::string { "完成: " } + task->meta.name);
                push_history(std::move(task));
            });
    }

    auto undo() noexcept -> std::future<bool> {
        auto promise = std::promise<bool> { };
        auto future  = promise.get_future();

        auto run_on_main = false;
        {
            auto lock = std::scoped_lock { history_guard };
            if (cursor > 0 && !history.empty()) {
                auto& record = history[cursor - 1];
                run_on_main  = record.event != nullptr && record.event->meta.main_thread;
            }
        }

        submit_on_executor(run_on_main,
            [this, promise = std::move(promise)]() mutable {
                auto lock = std::scoped_lock { history_guard };
                if (cursor == 0 || history.empty()) {
                    promise.set_value(false);
                    return;
                }

                auto& record = history[cursor - 1];
                if (record.event == nullptr) {
                    promise.set_value(false);
                    return;
                }

                spdlog::info("[runtime] undo: {}", record.event->meta.name);
                emit_operation(std::string { "撤销: " } + record.event->meta.name);
                record.event->undo();
                cursor -= 1;
                emit_operation(std::string { "已撤销: " } + record.event->meta.name);
                promise.set_value(true);
            });

        return future;
    }

    auto redo() noexcept -> std::future<bool> {
        auto promise = std::promise<bool> { };
        auto future  = promise.get_future();

        auto run_on_main = false;
        {
            auto lock = std::scoped_lock { history_guard };
            if (cursor < history.size()) {
                auto& record = history[cursor];
                run_on_main  = record.event != nullptr && record.event->meta.main_thread;
            }
        }

        submit_on_executor(run_on_main,
            [this, promise = std::move(promise)]() mutable {
                auto lock = std::scoped_lock { history_guard };
                if (cursor >= history.size()) {
                    promise.set_value(false);
                    return;
                }

                auto& record = history[cursor];
                if (record.event == nullptr || !record.event->meta.redoable) {
                    promise.set_value(false);
                    return;
                }

                spdlog::info("[runtime] redo: {}", record.event->meta.name);
                emit_operation(std::string { "重做: " } + record.event->meta.name);
                record.event->redo();
                cursor += 1;
                emit_operation(std::string { "已重做: " } + record.event->meta.name);
                promise.set_value(true);
            });

        return future;
    }

    auto can_undo() const noexcept -> bool {
        auto lock = std::scoped_lock { history_guard };
        return cursor > 0;
    }

    auto can_redo() const noexcept -> bool {
        auto lock = std::scoped_lock { history_guard };
        return cursor < history.size();
    }
};

Runtime::Runtime() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Runtime::~Runtime() noexcept = default;

auto Runtime::submit_internal(std::unique_ptr<runtime::internal::IEvent> task) noexcept -> void {
    pimpl->submit_task(std::move(task));
}

auto Runtime::set_operation_sink(std::function<void(std::string const&)> sink) noexcept -> void {
    auto lock            = std::scoped_lock { pimpl->sink_guard };
    pimpl->operation_sink = std::move(sink);
}

auto Runtime::undo() noexcept -> std::future<bool> { return pimpl->undo(); }

auto Runtime::redo() noexcept -> std::future<bool> { return pimpl->redo(); }

auto Runtime::can_undo() const noexcept -> bool { return pimpl->can_undo(); }

auto Runtime::can_redo() const noexcept -> bool { return pimpl->can_redo(); }

}
