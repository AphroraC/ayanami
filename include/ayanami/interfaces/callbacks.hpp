#pragma once

#include <string>

#include "ayanami/messages/actions.hpp"
#include "ayanami/messages/nav_msgs.hpp"
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
 *
 * 输入/输出的纪律: 本组全局槽只承载**输出/观测** (goal/feedback/result,
 * velocity command, costmap 可视化)。障碍观测 (pointcloud/laserscan) 这类
 * **输入**不走全局槽 —— 由 ROS 侧直接调 AbstractNavigation::setPointcloud /
 * setLaserScan → CostmapNavigationServer::handleObstacle* 落图 (宿主直递)。
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
  // ---- 状态 / 输出 / 观测 (spec §3.3 其余 4 个; on_robot_state_updated 无
  // 生产发射点, 已删除) ----
  static CallbackSlot<void(const ayanami::geometry_msgs::TwistStamped&)> on_velocity_command;
  static CallbackSlot<void(const std::string&, const ayanami::nav_msgs::OccupancyGrid&)> on_costmap_visualization;
};

} // namespace interfaces
} // namespace ayanami
