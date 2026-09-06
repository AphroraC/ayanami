#pragma once

#include "ayanami/interfaces/transform_store.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

#include <chrono>
#include <boost/circular_buffer.hpp>
#include <Eigen/Geometry>
#include <mutex>
#include <optional>
#include <string>

namespace ayanami {
namespace components {

/**
 * @brief 全局单例机器人状态控制器 (扩展)。
 *
 * 维护最新速度 (由 ayanami_ros odomCallback 直写) + 从 fast_tf TransformStore
 * 查询最新位姿 (global_frame ← robot_frame)。帧名与 velocity_timeout 只在
 * 一次性 initialize() 时读 Config 并缓存; hot getRobotState() 永不读 Config。
 *
 * 语义: 位姿恒取最新 TF; 速度取最新 odom, 距上次写入超过 velocity_timeout
 * 视为过期 → 返回零速度但位姿仍有效。
 *
 * 单例纪律 (同类): instance 与全部方法定义只在 libayanami 的
 * src/components/robot_state_controller.cpp 内 (不 header-inline), 使 ayanami_ros
 * 写者与核心 controller execution 读者跨 DSO 解析到同一实例。
 *
 * 线程安全: 全部公共方法经 mutex 串行化; getRobotState 锁内只做 TF 查询 +
 * 缓存拷贝, 不触发 IO/Config。
 */
class RobotStateController {
 public:
  static RobotStateController& instance();

  /// @brief 一次性初始化: 缓存帧名 + 速度超时 (hot path 不再读 Config)。
  /// @param local_frame 局部地图/局部视图解析机器人位姿的帧 (语义上通常是
  /// odom 帧; 与 costmap.local_frame 单一来源对齐, 非独立的 ROS TF
  /// odom 帧配置)。
  void initialize(const std::string& global_frame,
                  const std::string& local_frame,
                  const std::string& robot_frame, double velocity_timeout);

  /// @brief 更新最新速度 (ayanami_ros odomCallback 直写入口)。线程安全。
  void setVelocity(const geometry_msgs::TwistStamped& velocity);

  void setOdometry(const geometry_msgs::TwistStamped& odometry);
  bool getOdometry(const ayanami::Timestamp& stamp,
                   geometry_msgs::TwistStamped& odometry) const;
  void setTransform(const std::string& parent, const std::string& child,
                    ayanami::Timestamp stamp,
                    const Eigen::Isometry3d& transform,
                    bool is_static = false);
  bool lookupTransform(const std::string& target, const std::string& source,
                       ayanami::Timestamp stamp, Eigen::Isometry3d& transform,
                       double timeout = 0.0) const;

  /// @brief 读最新机器人状态: pose = TransformStore 最新 global←robot;
  /// velocity = 最新速度, 超过 velocity_timeout (或无速度) 时返回零速度。
  /// @return false 表示无可用位姿 (调用方按无位姿处理, 速度无效)。
  bool getRobotState(geometry_msgs::PoseStamped& pose,
                     geometry_msgs::TwistStamped& velocity) const;

  /// @brief 便捷访问器 (测试/观察); 语义同 getRobotState 对应字段。
  geometry_msgs::PoseStamped getRobotPose() const;
  geometry_msgs::TwistStamped getRobotVelocity() const;

  /// @brief 速度是否新鲜 (存在且在超时窗口内)。
  bool hasFreshVelocity() const;

  /// @brief 清空缓存速度 (测试用, 模拟超时)。
  void resetVelocity();

  std::string getGlobalFrame() const;
  std::string getLocalFrame() const;
  std::string getRobotFrame() const;
  double getVelocityTimeout() const;

 private:
  RobotStateController() = default;
  ~RobotStateController() = default;
  RobotStateController(const RobotStateController&) = delete;
  RobotStateController& operator=(const RobotStateController&) = delete;

  /// @brief 锁内调用: TransformStore 查询最新位姿 (global ← robot)。
  bool poseFromTransformStore(geometry_msgs::PoseStamped& pose) const;
  /// @brief 锁内调用: 最新速度 (过期则零速度)。
  geometry_msgs::TwistStamped velocityLocked() const;

  mutable std::mutex mutex;
  std::string global_frame{"map"};
  std::string local_frame{"odom"};
  std::string robot_frame{"base_link"};
  double velocity_timeout_s{0.25};
  std::optional<geometry_msgs::TwistStamped> last_velocity;
  std::chrono::steady_clock::time_point last_velocity_at{};
  boost::circular_buffer<geometry_msgs::TwistStamped> odometry_buffer{128};
  double odometry_time_tolerance_s{0.25};
};

}  // namespace components
}  // namespace ayanami
