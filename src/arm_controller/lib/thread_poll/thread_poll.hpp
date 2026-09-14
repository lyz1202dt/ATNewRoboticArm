// ThreadPool.h
#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

class ThreadPool {
public:
    // 构造：默认线程数为硬件并发数，0 表示自动
    explicit ThreadPool(size_t threads = 0)
        : stop_(false)
        , active_tasks_(0) {
        if (threads == 0) {
            threads = std::thread::hardware_concurrency();
            if (threads == 0)
                threads = 4; // 兜底
        }
        workers_.reserve(threads);
        for (size_t i = 0; i < threads; ++i) {
            workers_.emplace_back([this] { workerLoop(); });
        }
    }

    // 禁止拷贝
    ThreadPool(const ThreadPool&)            = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // 析构：等待所有任务完成
    ~ThreadPool() {
        shutdown();
    }

    // 提交任务，返回 future 以获取结果
    template <typename T, typename... Args>
    auto submit(T&& f, Args&&... args) -> std::future<std::invoke_result_t<T, Args...>> {

        using RetType = std::invoke_result_t<T, Args...>;

        // 用 shared_ptr 包装 packaged_task（因 packaged_task 不可拷贝）
        auto task = std::make_shared<std::packaged_task<RetType()>>(std::bind(std::forward<T>(f), std::forward<Args>(args)...));

        std::future<RetType> fut = task->get_future();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_) {
                throw std::runtime_error("submit on stopped ThreadPool");
            }
            tasks_.emplace([task]() { (*task)(); });
        }
        cond_.notify_one();
        return fut;
    }

    // 批量提交：减少加锁次数，提升吞吐
    template <typename T, typename... Args>
    void submitBatch(T&& f, Args&&... args) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_) {
                throw std::runtime_error("submit on stopped ThreadPool");
            }
            // C++17 折叠表达式
            (tasks_.emplace([f, a = std::forward<Args>(args)]() mutable { f(a); }), ...);
        }
        cond_.notify_all();
    }

    // 优雅关闭：等待队列中剩余任务完成
    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_)
                return;
            stop_ = true;
        }
        cond_.notify_all();
        for (auto& t : workers_) {
            if (t.joinable())
                t.join();
        }
    }

    // 等待队列中所有任务完成（不关闭线程池）
    void waitAll() {
        std::unique_lock<std::mutex> lock(mutex_);
        done_cond_.wait(lock, [this] { return tasks_.empty() && active_tasks_ == 0; });
    }

    size_t threadCount() const {
        return workers_.size();
    }

    size_t pendingTasks() {
        std::lock_guard<std::mutex> lock(mutex_);
        return tasks_.size();
    }

private:
    void workerLoop() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cond_.wait(lock, [this] { return stop_ || !tasks_.empty(); });

                // 关闭且队列为空 -> 退出
                if (stop_ && tasks_.empty())
                    return;

                task = std::move(tasks_.front());
                tasks_.pop();
                ++active_tasks_;
            }

            task();

            {
                std::lock_guard<std::mutex> lock(mutex_);
                --active_tasks_;
                if (tasks_.empty() && active_tasks_ == 0) {
                    done_cond_.notify_all();
                }
            }
        }
    }

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;

    std::mutex mutex_;
    std::condition_variable cond_;      // 通知 worker 取任务
    std::condition_variable done_cond_; // 通知 waitAll

    bool stop_;
    size_t active_tasks_;
};