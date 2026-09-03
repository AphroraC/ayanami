#pragma once

#include <chrono>
#include <cstdint>

namespace ayanami {

/**
 * @brief 中间件无关时间戳, 布局对齐 ROS2 builtin_interfaces/Time:
 *        {int32 sec, uint32 nanosec} (nanosec ∈ [0, 1e9), 负值地板归一,
 *        如 -1.7s = {sec: -2, nanosec: 3e8})。
 *
 * - 只依赖 std::chrono, 不引入 fast_tf / ROS 类型, 可跨中间件序列化。
 * - 语义与 fast_tf 对齐: {0, 0} == "latest" 哨兵 (查最新可用变换)。
 * - 生产代码应显式用 Timestamp::latest() 表达 "最新", 不依赖隐式零值。
 */
struct Timestamp {
  /// 秒分量 (ROS2: valid over all int32 values)。
  std::int32_t sec = 0;
  /// 纳秒分量, [0, 1e9)。
  std::uint32_t nanosec = 0;

  constexpr Timestamp() = default;

  constexpr explicit Timestamp(std::chrono::nanoseconds v) { fromNanoseconds(v.count()); }
  constexpr explicit Timestamp(std::int64_t ns_count) { fromNanoseconds(ns_count); }

  /// @brief 显式 latest sentinel (fast_tf: time_t{} 即最新)。
  static constexpr Timestamp latest() { return Timestamp{}; }

  constexpr bool isLatest() const { return sec == 0 && nanosec == 0; }

  double toSeconds() const {
    return static_cast<double>(count()) * 1e-9;
  }

  /// @brief 总纳秒数 (int64), 内部算术与 fast_tf 交互用。
  std::int64_t count() const {
    return static_cast<std::int64_t>(sec) * 1000000000LL +
           static_cast<std::int64_t>(nanosec);
  }

 private:
  constexpr void fromNanoseconds(std::int64_t total) {
    std::int64_t s = total / 1000000000LL;
    std::int64_t ns = total % 1000000000LL;
    if (ns < 0) {  // 负值地板归一 (ROS2 语义)
      ns += 1000000000LL;
      --s;
    }
    sec = static_cast<std::int32_t>(s);
    nanosec = static_cast<std::uint32_t>(ns);
  }
};

/// @brief 秒 (double) → Timestamp (spec/配置常以秒给时间)。
inline Timestamp timestampFromSeconds(double s) {
  return Timestamp{std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(s))};
}

}  // namespace ayanami
