#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

// This is not safe from ABA problem
// intended to push all the jobs in one thread and
// let multiple threads pop one by one (so no insert in between)
template <typename T>
struct LockFreeJobStack {
    LockFreeJobStack() = delete;
    LockFreeJobStack(size_t maxSize)
        : buffer(new T[maxSize]), top(buffer + maxSize), upper(buffer + maxSize) {
    }

    void push(T item) {
        while (true) {
            T* oldTop = top.load(std::memory_order_relaxed);
            T* newTop = oldTop - 1;
            if (top.compare_exchange_weak(oldTop, newTop)) {
                *newTop = item;
                return;
            }
        }
    }

    std::optional<T> pop() {
        while (true) {
            T* oldTop = top.load(std::memory_order_relaxed);
            if (oldTop == upper) {
                return std::nullopt;
            }
            T* newTop = oldTop + 1;
            if (top.compare_exchange_weak(oldTop, newTop)) {
                return *oldTop;
            }
        }
    }

    void clear() {
        while (true) {
            T* oldTop = top.load(std::memory_order_relaxed);
            T* newTop = upper;
            if (top.compare_exchange_weak(oldTop, newTop)) {
                return;
            }
        }
    }

  private:
    T* buffer;
    T* upper;
    std::atomic<T*> top;
};

template <typename T>
struct ThreadPool {
    ThreadPool(size_t threadNum, size_t maxJobs) : jobStack(maxJobs) {
        living = threadNum;
        while (threadNum--) {
            std::thread th(workerFunc(threadNum));
            th.detach();
        }
    }

    ~ThreadPool() {
        auto lock = std::unique_lock(mutex);
        stop = true;
        threadCond.notify_all();
        hostCond.wait(lock, [&]() { return this->living == 0; });
    }

    void setJobFunc(std::function<void(T)> jobFunc) {
        this->jobFunc = jobFunc;
    }

    void addJob(T job) {
        jobStack.push(job);
        ++nextPushIdx;
        nextPushIdx %= living;
    }

    void flush(size_t threadNum) {
        auto lock = std::unique_lock(mutex);
        requested = threadNum;
        threadCond.notify_all();
        hostCond.wait(lock, [&]() { return running == 0 && requested == 0; });
    }

  private:
    std::function<void()> workerFunc(int id) {
        return [&, id]() {
            while (true) {
                {
                    auto lock = std::unique_lock(mutex);
                    threadCond.wait(lock, [&]() { return requested || stop; });
                    if (stop) {
                        --living;
                        hostCond.notify_all();
                        return;
                    }
                    --requested;
                    ++running;
                    if (requested == 0) {
                        threadCond.notify_all();
                    } else {
                        threadCond.wait(lock, [&]() { return requested == 0; });
                    }
                }
                while (auto job = jobStack.pop()) {
                    jobFunc(*job);
                }
                auto lock = std::unique_lock(mutex);
                --running;
                if (running == 0) {
                    hostCond.notify_all();
                }
            }
        };
    }

    std::mutex mutex;
    std::condition_variable hostCond;
    std::condition_variable threadCond;
    LockFreeJobStack<T> jobStack;
    int requested{ 0 };
    int running{ 0 };
    int living{ 0 };
    int nextPushIdx{ 0 };
    bool stop{ false };
    std::function<void(T)> jobFunc;
};
