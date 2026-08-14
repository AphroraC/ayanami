#pragma once

#include <string>

#include "ayanami/messages/actions.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/messages/sensor_msgs.hpp"
#include "ayanami/utility/callback_slot.hpp"

namespace ayanami {
namespace interfaces {

/**
 * @brief 全局旁路回调槽聚合 (spec §3.3)。
 *
 * 槽对象静态成员只在 libayanami 的 src/interfaces/callbacks.cpp 内定义一份
 * (不 inline, spec §3.2 关键纪律): 否则每个 .so 各持一份实例, 插件 add 的
 * 回调与核心 emit 的回调落在不同对象上, 静默失效。
 *
 * 分工 (spec §3.4): 主流控制流走 BT 节点直接调 NavigationServerBase 的
 * TaskHandle 直连, 不走这里; 本组槽供旁路 (ROS 扩展 / 可视化 / 回退 /
 * 外部监控) 订阅。
 */
struct NavigationCallbacks {
  // ---- 四组件 action 流 (goal / feedback / result) ----
  static CallbackSlot<void(const ayanami::actions::GetPathGoal&)> on_get_path_goal;
  static CallbackSlot<void(const ayanami::actions::GetPathFeedback&)> on_get_path_feedback;
  static CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::GetPathResult>&)> on_get_path_result;
  static CallbackSlot<void(const ayanami::actions::SmoothPathGoal&)> on_smooth_path_goal;
  static CallbackSlot<void(const ayanami::actions::SmoothPathFeedback&)> on_smooth_path_feedback;
  static CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::SmoothPathResult>&)> on_smooth_path_result;
  static CallbackSlot<void(const ayanami::actions::ExePathGoal&)> on_exe_path_goal;
  static CallbackSlot<void(const ayanami::actions::ExePathFeedback&)> on_exe_path_feedback;
  static CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::ExePathResult>&)> on_exe_path_result;
  static CallbackSlot<void(const ayanami::actions::RecoveryGoal&)> on_recovery_goal;
  static CallbackSlot<void(const ayanami::actions::RecoveryFeedback&)> on_recovery_feedback;
  static CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::RecoveryResult>&)> on_recovery_result;
  // ---- 状态 / 输入 (spec §3.3 其余 5 个) ----
  static CallbackSlot<void(const ayanami::geometry_msgs::PoseStamped&, const ayanami::geometry_msgs::TwistStamped&)> on_robot_state_updated;
  static CallbackSlot<void(const ayanami::geometry_msgs::TwistStamped&)> on_velocity_command;
  static CallbackSlot<void(const std::string&, const ayanami::nav_msgs::OccupancyGrid&)> on_costmap_visualization;
  static CallbackSlot<void(const ayanami::sensor_msgs::PointCloud2&)> on_obstacle_pointcloud;
  static CallbackSlot<void(const ayanami::sensor_msgs::LaserScan&)> on_obstacle_laserscan;
};

} // namespace interfaces
} // namespace ayanami
