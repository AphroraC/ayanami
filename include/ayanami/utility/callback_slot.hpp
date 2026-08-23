#pragma once
#include <atomic>
#include <functional>
#include <shared_mutex>
#include <utility>
#include <vector>

namespace ayanami {

/**
 * @brief Multi-cast callback slot with runtime add/remove.
 *
 * Holds callbacks in a vector (holes are left as nullptr on remove to keep
 * IDs stable). All mutation (add/remove) takes the write lock; invocation
 * (call/operator()) takes the shared read lock so concurrent emission is
 * safe while subscriptions change.
 *
 * @note The static slot objects that need to be shared across .so boundaries
 *       (see NavigationCallbacks) must be defined in libayanami's .cpp, NOT
 *       as inline/static members of a header — otherwise each .so keeps its
 *       own instance and callbacks registered from plugins silently vanish.
 */
template <typename Func>
class CallbackSlot {
  std::vector<std::function<Func>> callbacks_;
  mutable std::shared_mutex mutex_;
  std::atomic<std::size_t> subscriber_count_{0};

public:
  /**
   * @brief Register a callback, returning its stable id (vector index).
   */
  int add(std::function<Func> cb) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    callbacks_.push_back(std::move(cb));
    subscriber_count_.fetch_add(1, std::memory_order_release);
    return static_cast<int>(callbacks_.size() - 1);
  }

  /**
   * @brief Remove the callback at @p id (leaves a nullptr hole).
   */
  void remove(int id) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (id >= 0 && id < static_cast<int>(callbacks_.size())) {
      auto& callback = callbacks_[static_cast<std::size_t>(id)];
      if (callback) {
        callback = nullptr;
        subscriber_count_.fetch_sub(1, std::memory_order_release);
      }
    }
  }

  /**
   * @brief Invoke every non-null callback, forwarding @p args.
   */
  template <typename... Args>
  void call(Args&&... args) const {
    if (subscriber_count_.load(std::memory_order_acquire) == 0) {
      return;
    }

    std::vector<std::function<Func>> callbacks;
    {
      std::shared_lock<std::shared_mutex> lock(mutex_);
      callbacks.reserve(subscriber_count_.load(std::memory_order_relaxed));
      for (const auto& cb : callbacks_) {
        if (cb) {
          callbacks.push_back(cb);
        }
      }
    }

    // Invoke outside the lock so callbacks may add/remove subscriptions or
    // emit the same slot recursively. The snapshot defines this emission.
    for (const auto& cb : callbacks) {
      cb(std::forward<Args>(args)...);
    }
  }

  /**
   * @brief Operator form of call().
   */
  template <typename... Args>
  void operator()(Args&&... args) const {
    call(std::forward<Args>(args)...);
  }

  /**
   * @brief True if at least one non-null callback is registered.
   */
  explicit operator bool() const {
    return subscriber_count_.load(std::memory_order_acquire) != 0;
  }
};

} // namespace ayanami
