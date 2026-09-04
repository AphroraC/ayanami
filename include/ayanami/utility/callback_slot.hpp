#pragma once
#include <atomic>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>
#include <vector>

namespace ayanami {

/**
 * @brief Exception handler installed for observer callbacks.
 *
 * A process-wide hook reported to whenever an external callback throws during
 * emission. Keeping it a plain function pointer keeps this header free of any
 * logging framework; libayanami (or the host application) can install an
 * spdlog-backed handler via setCallbackExceptionHandler() without dragging
 * spdlog into every TU that includes this template header. Default: nullptr,
 * i.e. thrown exceptions are swallowed silently (documented policy).
 *
 * @note The hook may be installed only from code, not from a header initializer.
 */
using callback_exception_handler = void (*)(std::exception_ptr) noexcept;

/** RAII handle for one callback subscription. */
class CallbackSubscription {
 public:
  CallbackSubscription() = default;
  explicit CallbackSubscription(std::function<void()> reset)
      : reset_(std::move(reset)) {}
  ~CallbackSubscription() { reset(); }

  CallbackSubscription(const CallbackSubscription&) = delete;
  CallbackSubscription& operator=(const CallbackSubscription&) = delete;
  CallbackSubscription(CallbackSubscription&& other) noexcept
      : reset_(std::move(other.reset_)) {}
  CallbackSubscription& operator=(CallbackSubscription&& other) noexcept {
    if (this != &other) {
      reset();
      reset_ = std::move(other.reset_);
    }
    return *this;
  }

  void reset() noexcept {
    if (!reset_) return;
    auto reset = std::move(reset_);
    try {
      reset();
    } catch (...) {
      // Unsubscription must never throw during object destruction.
    }
  }

 private:
  std::function<void()> reset_;
};

namespace detail {

inline std::atomic<callback_exception_handler>& callbackExceptionSink() {
  static std::atomic<callback_exception_handler> sink{nullptr};
  return sink;
}

}  // namespace detail

/**
 * @brief Install (or clear) the process-wide callback exception handler.
 */
inline void setCallbackExceptionHandler(callback_exception_handler handler) noexcept {
  detail::callbackExceptionSink().store(handler, std::memory_order_release);
}

/**
 * @brief Return the currently installed callback exception handler.
 */
inline callback_exception_handler getCallbackExceptionHandler() noexcept {
  return detail::callbackExceptionSink().load(std::memory_order_acquire);
}

/**
 * @brief Report (or swallow) the exception currently being handled.
 *
 * Called from the catch(...) of an emitting thread. Guarded so that even a
 * misbehaving logger installed as the sink cannot escape and terminate/abort
 * the controller loop.
 */
inline void reportCallbackException() noexcept {
  try {
    callback_exception_handler sink =
        detail::callbackExceptionSink().load(std::memory_order_relaxed);
    if (!sink) {
      return;
    }
    std::exception_ptr ep = std::current_exception();
    if (ep) {
      sink(ep);
    }
  } catch (...) {
    // Nothing sane left to do; the exception is still swallowed below.
  }
}

/**
 * @brief Multi-cast callback slot with runtime add/remove, tuned for
 *        high-frequency emission.
 *
 * Holds a copy-on-write immutable snapshot of the callback list. Emissions
 * only read the currently published snapshot via an atomic load: no mutex, no
 * allocation, no std::function copies on the call path — each callback is
 * dereferenced in place. Mutations (add/remove) serialize on an internal
 * mutex, copy the snapshot, apply the change and republish it; concurrent
 * emitters keep the old snapshot alive for the duration of their call.
 *
 * Stable integer IDs/holes are preserved exactly as before: add() appends and
 * returns the index, remove() leaves a hole, IDs never get reused.
 * Because every emission locks a snapshot at entry, reentrant emit/add/remove
 * (a callback adding/removing subscriptions or emitting the same slot while
 * running) affects subsequent emissions, never the one in flight.
 *
 * Exceptions thrown by observer callbacks are caught per callback and logged
 * via the installed callback_exception_handler (default: swallowed), so one
 * broken observer cannot interrupt the remaining observers or the controller
 * execution.
 *
 * @note The static slot objects that need to be shared across .so boundaries
 *       (see NavigationCallbacks) must be defined in libayanami's .cpp, NOT
 *       as inline/static members of a header — otherwise each .so keeps its
 *       own instance and callbacks registered from plugins silently vanish.
 *       The exception-handler hook above has the same caveat: set it from the
 *       main DSO if plugins should adopt the same logging sink.
 */
template <typename Func>
class CallbackSlot {
  using CallbackList = std::vector<std::function<Func>>;

  std::shared_ptr<const CallbackList> snapshot;
  std::atomic<std::size_t> subscriber_count{0};
  // 缓存行隔离: add/remove 写 mutex 时不与 emit 热读的
  // snapshot/subscriber_count 产生伪共享 (对齐到独立 cache line)。
  alignas(64) mutable std::mutex mutex;

  void publish(std::shared_ptr<CallbackList> fresh) {
    std::atomic_store_explicit(
        &snapshot, std::shared_ptr<const CallbackList>(std::move(fresh)),
        std::memory_order_release);
  }

public:
  /**
   * @brief Register a callback, returning its stable id (vector index).
   */
  int add(std::function<Func> cb) {
    std::lock_guard<std::mutex> lock(mutex);
    std::shared_ptr<const CallbackList> current =
        std::atomic_load(&snapshot);
    auto fresh = std::make_shared<CallbackList>();
    if (current) {
      fresh->reserve(current->size() + 1);
      fresh->insert(fresh->end(), current->begin(), current->end());
    }
    const int id = static_cast<int>(fresh->size());
    fresh->emplace_back(std::move(cb));
    publish(std::move(fresh));
    subscriber_count.fetch_add(1, std::memory_order_release);
    return id;
  }

  /** Register a callback whose lifetime is tied to the returned handle. */
  CallbackSubscription subscribe(std::function<Func> cb) {
    const int id = add(std::move(cb));
    return CallbackSubscription([this, id] { remove(id); });
  }

  /**
   * @brief Remove the callback at @p id (leaves a nullptr hole).
   */
  void remove(int id) {
    std::lock_guard<std::mutex> lock(mutex);
    if (id < 0) {
      return;
    }
    const std::size_t idx = static_cast<std::size_t>(id);
    std::shared_ptr<const CallbackList> current =
        std::atomic_load(&snapshot);
    if (!current || idx >= current->size() || !(*current)[idx]) {
      return;
    }
    auto fresh = std::make_shared<CallbackList>(*current);
    (*fresh)[idx] = nullptr;
    publish(std::move(fresh));
    subscriber_count.fetch_sub(1, std::memory_order_release);
  }

  /**
   * @brief Invoke every non-null callback, forwarding @p args.
   *
   * Lock-free: only atomic-loads the published snapshot. Callbacks are run
   * from the immutable snapshot, so a callback may add/remove subscriptions
   * or emit the same slot recursively without affecting this emission.
   *
   * 右值防护 (编译期): 参数会转发给每一个 observer, 非平凡类型的右值/按值
   * 参数会在第一个 observer 处被 move 搬空, 后续 observer 读到 moved-from
   * 状态。因此只允许左值引用或 trivially-copyable 的按值实参 (对可平凡拷贝
   * 类型 move == copy, 每个 observer 都拿到完整值); 非平凡结构体请传左值。
   */
  template <typename... Args>
  void call(Args&&... args) const {
    static_assert(
        (... && (std::is_lvalue_reference_v<Args> ||
                std::is_trivially_copyable_v<std::remove_reference_t<Args>>)),
        "CallbackSlot::call forwards arguments to every observer: pass "
        "lvalue references or trivially-copyable values; a non-trivial "
        "rvalue argument would be moved-from after the first observer");
    if (subscriber_count.load(std::memory_order_acquire) == 0) {
      return;
    }
    std::shared_ptr<const CallbackList> snap =
        std::atomic_load(&snapshot);
    if (!snap) {
      return;
    }
    for (const auto& cb : *snap) {
      if (!cb) {
        continue;
      }
      try {
        cb(std::forward<Args>(args)...);
      } catch (...) {
        reportCallbackException();
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
    return subscriber_count.load(std::memory_order_acquire) != 0;
  }
};

}  // namespace ayanami
