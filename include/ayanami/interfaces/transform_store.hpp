#pragma once

#include <memory>
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
  ///
  /// 时间语义: latest 哨兵 ({0,0}) 在多跳链路上解析为 latest common time
  /// (tf2 语义: 公共路径上各动态边最新样本戳的最小值, 统一切刻采样);
  /// 单边查询即该边最新样本。具体戳在动态边缓存窗口内插值, 窗口外
  /// (双向外推) 失败。
  ///
  /// @return true on success (fills @p out), false if no transform available.
  bool lookupTransform(const std::string& target, const std::string& source,
                       Timestamp stamp, Eigen::Isometry3d& out,
                       double timeout = 0.0) const;

  /// @brief 配置动态边的按戳回溯窗口 (秒; fast_tf 剪枝保留
  /// [最新样本 - 窗口, 最新样本])。未配置时保持 fast_tf 默认 500ms。
  ///
  /// 仅限启动期调用 (AyanamiROS 构造 / 测试注入): 实现为整 buffer 替换,
  /// 已存 TF 数据会被丢弃; 不允许与并发 set/lookup 并发。生产路径在
  /// AyanamiROS 构造函数中、任何 TF 写入之前调用一次。
  void configureCacheDuration(double seconds);

  /// @brief 查询 target/source 链路的 latest common time (诊断/测试面)。
  /// @param out 链路公共最新时刻; epoch 零值 ({0,0}, 即 latest 哨兵) 表示
  /// 链上无任何动态边持有数据。
  /// @return false 表示帧未知或两帧不在同一棵树。
  bool latestCommonTime(const std::string& target, const std::string& source,
                        Timestamp& out) const;

  /// @brief 兼容 helper: 秒 since epoch → Timestamp。
  /// 生产代码查询 "最新" 请显式用 Timestamp::latest, 勿用 fromSeconds(0)。
  static Timestamp fromSeconds(double s);

private:
  TransformStore() = default;
  ~TransformStore() = default;
  TransformStore(const TransformStore&) = delete;
  TransformStore& operator=(const TransformStore&) = delete;

  /// @brief 底层 buffer。unique_ptr 间接持有: transform_buffer 含 mutex
  /// 不可赋值, configureCacheDuration 经整对象替换实现 (启动期专用)。
  std::unique_ptr<fast_tf::detail::transform_buffer> buffer{
      std::make_unique<fast_tf::detail::transform_buffer>()};
};

}  // namespace interfaces
}  // namespace ayanami
