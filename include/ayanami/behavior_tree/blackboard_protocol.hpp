#pragma once

#include <behaviortree_cpp/blackboard.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace ayanami {
namespace behavior_tree {

// 黑板协议集中常量 (spec §6.5): 所有端口/黑名单键名唯一来源, 把运行时端口名错误
// 变成编译期/启动期失败。
inline constexpr const char* kGoal = "goal";
inline constexpr const char* kRawPath = "raw_path";
inline constexpr const char* kSmoothedPath = "smoothed_path";
inline constexpr const char* kPlannerFrequency = "planner_frequency";
inline constexpr const char* kControllerFrequency = "controller_frequency";
inline constexpr const char* kPlannerId = "planner_id";
inline constexpr const char* kSmootherId = "smoother_id";
inline constexpr const char* kControllerId = "controller_id";
inline constexpr const char* kRecoveryId = "recovery_id";
inline constexpr const char* kClearCostmapId = "clear_costmap_id";
inline constexpr const char* kSpinId = "spin_id";
inline constexpr const char* kWaitId = "wait_id";
inline constexpr const char* kBackupId = "backup_id";
inline constexpr const char* kSessionId = "session_id";
inline constexpr const char* kPlanVersion = "plan_version";
inline constexpr const char* kResetStuckToken = "reset_stuck_token";

/// 全部必需黑板项 (createTree 之后 validateRequiredBlackboardEntries 扫一遍)。
inline const std::vector<const char*>& requiredBlackboardEntries() {
  static const std::vector<const char*> kRequired = {
    kGoal, kRawPath, kSmoothedPath, kPlannerFrequency, kControllerFrequency,
    kPlannerId, kSmootherId, kControllerId, kRecoveryId, kClearCostmapId,
    kSpinId, kWaitId, kBackupId, kSessionId, kPlanVersion, kResetStuckToken,
  };
  return kRequired;
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
