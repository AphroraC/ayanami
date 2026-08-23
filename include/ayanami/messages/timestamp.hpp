#pragma once

#include <chrono>
#include <cstdint>

namespace ayanami {

/**
 * @brief 中间件无关时间戳 (自 epoch 起的纳秒数)。
 *
 * - 只依赖 std::chrono, 不引入 fast_tf / ROS 类型, 可跨中间件序列化。
 * - 语义与 fast_tf 对齐: 0 纳秒 == "latest" 哨兵 (查最新可用变换)。
 * - 生产代码应显式用 Timestamp::latest() 表达 "最新", 不依赖隐式 0 值。
 */
struct Timestamp {
  std::chrono::nanoseconds ns{0};

  constexpr Timestamp() = default;
  constexpr explicit Timestamp(std::chrono::nanoseconds v) : ns(v) {}
  constexpr explicit Timestamp(std::int64_t ns_count) : ns(ns_count) {}

  /// @brief 显式 latest sentinel (fast_tf: time_t{} 即最新)。
  static constexpr Timestamp latest() { return Timestamp{}; }

  constexpr bool isLatest() const { return ns.count() == 0; }

  double toSeconds() const {
    return std::chrono::duration<double>(ns).count();
  }

  std::int64_t count() const { return ns.count(); }
};

/// @brief 秒 (double) → Timestamp (spec/配置常以秒给时间)。
inline Timestamp timestampFromSeconds(double s) {
  return Timestamp{std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(s))};
}

}  // namespace ayanami