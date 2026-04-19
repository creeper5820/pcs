#include "runtime.hh"

#include <qcoreapplication.h>
#include <qmetaobject.h>
#include <qrunnable.h>
#include <qthreadpool.h>

namespace pcs {

class MainExecutor : public co::runtime_executor {
public:
    auto submit_task(std::unique_ptr<task> task) noexcept -> void override {
        QMetaObject::invokeMethod(
            QCoreApplication::instance(), [task = std::move(task)] { task->run(); },
            Qt::QueuedConnection);
    }
};
class WorkExecutor : public co::runtime_executor {
public:
    auto submit_task(std::unique_ptr<task> task) noexcept -> void override {
        struct TaskRunnable : QRunnable {
            explicit TaskRunnable(std::unique_ptr<co::runtime_executor::task> t) noexcept
                : task { std::move(t) } {
                setAutoDelete(true);
            }

            auto run() -> void override { task->run(); }

            std::unique_ptr<co::runtime_executor::task> task;
        };

        QThreadPool::globalInstance()->start(new TaskRunnable { std::move(task) });
    }
};

struct Runtime::Impl {

    std::unique_ptr<MainExecutor> main_exec;
    std::unique_ptr<WorkExecutor> work_exec;

    explicit Impl() {
        main_exec = std::make_unique<MainExecutor>();
        work_exec = std::make_unique<WorkExecutor>();
    }

    auto submit_task(std::unique_ptr<Task> task) noexcept {
        static_cast<co::runtime_executor&>(*work_exec)
            .submit_task([task = std::move(task)]() mutable { task->exec(); });
    }
};

Runtime::Runtime() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Runtime::~Runtime() noexcept = default;

auto Runtime::submit(std::unique_ptr<Task> task) noexcept -> void {
    pimpl->submit_task(std::move(task));
}

}
