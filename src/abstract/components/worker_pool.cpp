#include "ayanami/abstract/components/worker_pool.hpp"

#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace ayanami {
namespace components {

namespace {

/// @brief 默认 worker 数; 可用 AYANAMI_POOL_THREADS 覆盖 (1..64)。
std::size_t desiredWorkerCount() {
  std::size_t n = 8;
  if (const char* env = std::getenv("AYANAMI_POOL_THREADS")) {
    const int parsed = std::atoi(env);
    if (parsed > 0) {
      n = static_cast<std::size_t>(parsed);
    }
  }
  return n > 64 ? 64 : n;
}

} // namespace

class WorkerPool::Impl {
public:
  explicit Impl(std::size_t n) {
    workers.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      workers.emplace_back([this] { workerLoop(); });
    }
  }

  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopping = true;
    }
    cv.notify_all();
    for (auto& w : workers) {
      if (w.joinable()) {
        w.join();
      }
    }
    // 优雅排空: join 之后队列遗留任务在析构线程尽力执行 (正常路径队列已空)。
    while (true) {
      std::function<void()> task;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.empty()) {
          break;
        }
        task = std::move(queue.front());
        queue.pop_front();
      }
      try {
        task();
      } catch (...) {
      }
    }
  }

  void submit(std::function<void()> task) {
    if (!task) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (stopping) {
        // 停池后丢弃 (组件析构应先等齐其任务, 正常路径不触发)。
        return;
      }
      queue.push_back(std::move(task));
    }
    cv.notify_one();
  }

  std::size_t queuedCount() const {
    std::lock_guard<std::mutex> lock(mutex);
    return queue.size();
  }

  void shutdown() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopping = true;
    }
    cv.notify_all();
    for (auto& w : workers) {
      if (w.joinable()) {
        w.join();
      }
    }
  }

private:
  void workerLoop() {
    while (true) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this] { return stopping || !queue.empty(); });
        if (stopping && queue.empty()) {
          return;
        }
        task = std::move(queue.front());
        queue.pop_front();
      }
      try {
        task();
      } catch (...) {
        // 任务异常吸收; 不因单个任务拖垮池。
      }
    }
  }

  mutable std::mutex mutex;
  std::condition_variable cv;
  std::deque<std::function<void()>> queue;
  bool stopping{false};
  std::vector<std::thread> workers;
};

WorkerPool::WorkerPool()
    : impl_(std::make_unique<Impl>(desiredWorkerCount())) {}

WorkerPool::~WorkerPool() = default;

WorkerPool& WorkerPool::instance() {
  static WorkerPool pool;
  return pool;
}

void WorkerPool::submit(std::function<void()> task) {
  impl_->submit(std::move(task));
}

std::size_t WorkerPool::queuedCount() const {
  return impl_->queuedCount();
}

void WorkerPool::shutdown() { impl_->shutdown(); }

} // namespace components
} // namespace ayanami