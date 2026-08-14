#pragma once
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

public:
  /**
   * @brief Register a callback, returning its stable id (vector index).
   */
  int add(std::function<Func> cb) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    callbacks_.push_back(std::move(cb));
    return static_cast<int>(callbacks_.size() - 1);
  }

  /**
   * @brief Remove the callback at @p id (leaves a nullptr hole).
   */
  void remove(int id) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (id >= 0 && id < static_cast<int>(callbacks_.size())) {
      callbacks_[static_cast<std::size_t>(id)] = nullptr;
    }
  }

  /**
   * @brief Invoke every non-null callback, forwarding @p args.
   */
  template <typename... Args>
  void call(Args&&... args) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    for (const auto& cb : callbacks_) {
      if (cb) {
        cb(std::forward<Args>(args)...);
      }
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
    std::shared_lock<std::shared_mutex> lock(mutex_);
    for (const auto& cb : callbacks_) {
      if (cb) {
        return true;
      }
    }
    return false;
  }
};

} // namespace ayanami
