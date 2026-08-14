#pragma once

#include <chrono>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fast_tf.hpp>

namespace ayanami {
namespace interfaces {

/**
 * @brief Thread-safe singleton wrapping fast_tf::detail::transform_buffer.
 *
 * Holds the transform tree (static + timed dynamic links) in one global
 * buffer. Written by the ROS sync layer (periodic + sensor callbacks), read by
 * the pure C++ core (frame transforms, costmap frame, BT robot pose).
 */
class TransformStore {
public:
  /// @brief Timestamp type (fast_tf system-clock time point).
  using TimePoint = fast_tf::detail::time_t;

  /// @brief Global singleton instance.
  static TransformStore& instance();

  /// @brief Insert a transform (parent -> child).
  void setTransform(const std::string& parent, const std::string& child,
                    TimePoint stamp, const Eigen::Isometry3d& T,
                    bool is_static = false);

  /// @brief Look up the transform from @p source frame to @p target frame.
  /// @return true on success (fills @p out), false if no transform available.
  bool lookupTransform(const std::string& target, const std::string& source,
                       TimePoint stamp, Eigen::Isometry3d& out,
                       double timeout = 0.0) const;

  /// @brief Convert seconds-since-epoch to a TimePoint.
  static TimePoint fromSeconds(double s);

private:
  TransformStore() = default;
  ~TransformStore() = default;
  TransformStore(const TransformStore&) = delete;
  TransformStore& operator=(const TransformStore&) = delete;

  fast_tf::detail::transform_buffer buffer_;
};

inline TransformStore&
TransformStore::instance() {
  static TransformStore store;
  return store;
}

inline void
TransformStore::setTransform(const std::string& parent,
                             const std::string& child, TimePoint stamp,
                             const Eigen::Isometry3d& T, bool is_static) {
  buffer_.set(parent, child, stamp, T, is_static);
}

inline bool
TransformStore::lookupTransform(const std::string& target,
                                const std::string& source, TimePoint stamp,
                                Eigen::Isometry3d& out, double timeout) const {
  const auto tolerance = std::chrono::duration_cast<fast_tf::detail::duration_t>(
      std::chrono::duration<double>(timeout));
  try {
    out = buffer_.get(target, source, stamp, tolerance);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

inline TransformStore::TimePoint
TransformStore::fromSeconds(double s) {
  return TimePoint{std::chrono::duration_cast<fast_tf::detail::duration_t>(
      std::chrono::duration<double>(s))};
}

}  // namespace interfaces
}  // namespace ayanami
