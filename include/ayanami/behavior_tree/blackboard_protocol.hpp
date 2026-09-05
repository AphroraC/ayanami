#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

#include <behaviortree_cpp/blackboard.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace ayanami {
namespace behavior_tree {

/// @brief goal 等价判定 (容忍位姿数值噪声): stamp/frame_id + position/orientation
/// isApprox。旧 header.seq 已随 ROS2 Header 布局移除, 目标变化检测统一走本函数。
inline bool goalEquals(const geometry_msgs::PoseStamped& lhs,
                       const geometry_msgs::PoseStamped& rhs) {
  if (lhs.header.stamp.count() != rhs.header.stamp.count()) {
    return false;
  }
  if (lhs.header.frame_id != rhs.header.frame_id) {
    return false;
  }
  return lhs.pose.position.isApprox(rhs.pose.position) &&
         lhs.pose.orientation.isApprox(rhs.pose.orientation);
}

// 黑板协议集中常量 (spec §6.5): 所有端口/黑名单键名唯一来源, 把运行时端口名错误
// 变成编译期/启动期失败。
inline constexpr const char* k_goal = "goal";
inline constexpr const char* k_raw_path = "raw_path";
inline constexpr const char* k_path_id = "path_id";
inline constexpr const char* k_smoothed_path = "smoothed_path";
inline constexpr const char* k_planner_frequency = "planner_frequency";
inline constexpr const char* k_smoother_frequency = "smoother_frequency";
inline constexpr const char* k_controller_frequency = "controller_frequency";
inline constexpr const char* k_clear_costmap_id = "clear_costmap_id";
inline constexpr const char* k_spin_id = "spin_id";
inline constexpr const char* k_wait_id = "wait_id";
inline constexpr const char* k_backup_id = "backup_id";
inline constexpr const char* k_session_id = "session_id";
inline constexpr const char* k_plan_version = "plan_version";
inline constexpr const char* k_goal_revision = "goal_revision";
inline constexpr const char* k_reset_stuck_token = "reset_stuck_token";
inline constexpr const char* k_failure_context = "failure_context";

/// 全部必需黑板项 (createTree 之后 validateRequiredBlackboardEntries 扫一遍)。
inline const std::vector<const char*>& requiredBlackboardEntries() {
  static const std::vector<const char*> k_required = {
    k_goal, k_raw_path, k_path_id, k_smoothed_path, k_planner_frequency, k_smoother_frequency,
    k_controller_frequency,
    k_clear_costmap_id,
    k_spin_id, k_wait_id, k_backup_id, k_session_id, k_plan_version, k_reset_stuck_token,
    k_failure_context, k_goal_revision,
  };
  return k_required;
}

/// 校验协议要求的黑板项全部存在, 缺失立即 throw (spec §6.5)。
inline void validateRequiredBlackboardEntries(const BT::Blackboard::Ptr& blackboard) {
  if (!blackboard) {
    throw std::runtime_error("blackboard_protocol: blackboard is null");
  }
  for (const char* key : requiredBlackboardEntries()) {
    if (!blackboard->getEntry(key)) {
      throw std::runtime_error(
          std::string("blackboard_protocol: required entry missing: ") + key);
    }
  }
}

} // namespace behavior_tree
} // namespace ayanami
