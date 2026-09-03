#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fast_tf.hpp>

#include "ayanami/messages/timestamp.hpp"

namespace ayanami {
namespace interfaces {

/**
 * @brief Thread-safe singleton wrapping fast_tf::detail::transform_buffer.
 *
 * Holds the transform tree (static + timed dynamic links) in one global
 * buffer. Written by the ROS sync layer (periodic + sensor callbacks), read by
 * the pure C++ core (frame transforms, costmap frame, BT robot pose).
 *
 * 跨 DSO 单例纪律: instance() 与所有成员定义在 src/interfaces/
 * transform_store.cpp (非 inline), 保证多 .so 解析到同一实例。
 *
 * 时间语义 (Header timestamp / TF 时间通语义):
 * - 对外统一用中间件无关 ayanami::Timestamp (纳秒), 内部转 fast_tf::time_t;
 *   fast_tf 类型不泄漏到 messages / 调用方。
 * - "最新" 用显式 ayanami::Timestamp::latest() 哨兵, 不再依赖
 *   fromSeconds(0.0) 隐式 latest (fromSeconds 仅保留作兼容 helper)。
 */
class TransformStore {
public:
  /// @brief 中间件无关时间戳 (内部与 fast_tf::detail::time_t 互转)。
  using Timestamp = ayanami::Timestamp;

  /// @brief Global singleton instance.
  static TransformStore& instance();

  /// @brief Insert a transform (parent -> child).
  void setTransform(const std::string& parent, const std::string& child,
                    Timestamp stamp, const Eigen::Isometry3d& T,
                    bool is_static = false);

  /// @brief Look up the transform from @p source frame to @p target frame.
  /// @return true on success (fills @p out), false if no transform available.
  bool lookupTransform(const std::string& target, const std::string& source,
                       Timestamp stamp, Eigen::Isometry3d& out,
                       double timeout = 0.0) const;

  /// @brief 兼容 helper: 秒 since epoch → Timestamp。
  /// 生产代码查询 "最新" 请显式用 Timestamp::latest(), 勿用 fromSeconds(0)。
  static Timestamp fromSeconds(double s);

private:
  TransformStore() = default;
  ~TransformStore() = default;
  TransformStore(const TransformStore&) = delete;
  TransformStore& operator=(const TransformStore&) = delete;

  fast_tf::detail::transform_buffer buffer;
};

}  // namespace interfaces
}  // namespace ayanami
