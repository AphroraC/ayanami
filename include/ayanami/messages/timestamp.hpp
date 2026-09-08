#pragma once

#include <chrono>
#include <cstdint>

namespace ayanami {

/**
 * @brief Middleware-agnostic timestamp, layout aligned with ROS2
 *        builtin_interfaces/Time:
 *        {int32 sec, uint32 nanosec} (nanosec in [0, 1e9), negative values
 *        floor-normalized, e.g. -1.7s = {sec: -2, nanosec: 3e8}).
 *
 * - Depends only on std::chrono; no fast_tf / ROS types introduced; can be
 *   serialized across middlewares.
 * - Semantics aligned with fast_tf: {0, 0} == the "latest" sentinel (query
 *   the newest available transform).
 * - Production code should express "latest" explicitly with
 *   Timestamp::latest() and not rely on the implicit zero value.
 */
struct Timestamp {
  /// Seconds component (ROS2: valid over all int32 values).
  std::int32_t sec = 0;
  /// Nanoseconds component, [0, 1e9).
  std::uint32_t nanosec = 0;

  constexpr Timestamp() = default;

  constexpr explicit Timestamp(std::chrono::nanoseconds v) { fromNanoseconds(v.count()); }
  constexpr explicit Timestamp(std::int64_t ns_count) { fromNanoseconds(ns_count); }

  /// @brief Explicit latest sentinel (in fast_tf, time_t{} means latest).
  static constexpr Timestamp latest() { return Timestamp{}; }

  constexpr bool isLatest() const { return sec == 0 && nanosec == 0; }

  double toSeconds() const {
    return static_cast<double>(count()) * 1e-9;
  }

  /// @brief Total nanoseconds (int64), for internal arithmetic and fast_tf interactions.
  std::int64_t count() const {
    return static_cast<std::int64_t>(sec) * 1000000000LL +
           static_cast<std::int64_t>(nanosec);
  }

 private:
  constexpr void fromNanoseconds(std::int64_t total) {
    std::int64_t s = total / 1000000000LL;
    std::int64_t ns = total % 1000000000LL;
    if (ns < 0) {  // floor-normalize negative values (ROS2 semantics)
      ns += 1000000000LL;
      --s;
    }
    sec = static_cast<std::int32_t>(s);
    nanosec = static_cast<std::uint32_t>(ns);
  }
};

/// @brief Seconds (double) -> Timestamp (specs/configs usually give time in seconds).
inline Timestamp timestampFromSeconds(double s) {
  return Timestamp{std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(s))};
}

}  // namespace ayanami
