#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

#include <behaviortree_cpp/blackboard.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace ayanami {
namespace behavior_tree {

/// @brief Goal equivalence check (tolerates pose numeric noise):
/// stamp/frame_id + position/orientation isApprox. The old header.seq was
/// removed along with the ROS2 Header layout; goal-change detection goes
/// uniformly through this function.
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

// Centralized blackboard protocol constants: the single source of all
// port/blackboard key names, turning runtime port-name errors into
// compile-time/startup failures. The recovery role-id keys
// (k_clear_costmap_id / k_spin_id / k_wait_id / k_backup_id) are part of the
// default BT extension's tree contract, not of the core protocol: they are
// declared+provided by the extension that references them.
inline constexpr const char* k_goal = "goal";
inline constexpr const char* k_raw_path = "raw_path";
inline constexpr const char* k_smoothed_path = "smoothed_path";
inline constexpr const char* k_planner_frequency = "planner_frequency";
inline constexpr const char* k_smoother_frequency = "smoother_frequency";
inline constexpr const char* k_controller_frequency = "controller_frequency";
inline constexpr const char* k_clear_costmap_id = "clear_costmap_id";
inline constexpr const char* k_spin_id = "spin_id";
inline constexpr const char* k_wait_id = "wait_id";
inline constexpr const char* k_backup_id = "backup_id";
inline constexpr const char* k_session_id = "session_id";
// The producer of a path, at the granularity the tree needs to react to:
// ComputePathToPose advances raw_path_revision when it lands a new raw path,
// and the smoother branch keys its RateController reset_token off that key.
// Without a separate revision the smoother would have to watch the live plan
// version, which it advances itself -- so it could not tell "a new raw path
// landed" from "I just smoothed". Two producers, two revisions.
//
// There is deliberately no blackboard mirror of the live plan version: the
// single authority is NavigationSessionState::currentPlanVersion() (readable
// from any node through the injected NavigationRuntime). A second, key-shaped
// copy is how the two-plan-versions confusion started.
inline constexpr const char* k_raw_path_revision = "raw_path_revision";
inline constexpr const char* k_goal_revision = "goal_revision";
inline constexpr const char* k_failure_context = "failure_context";
// Outer RecoveryNode retry budget, injected from navigation.max_nav_retries
// (config owns the policy; the tree XML only binds the port to this key).
inline constexpr const char* k_nav_retries = "nav_retries";
// Navigation-level oscillation trigger parameters (MBF semantics),
// injected from navigation.oscillation_{distance,timeout}.
inline constexpr const char* k_oscillation_distance = "oscillation_distance";
inline constexpr const char* k_oscillation_timeout = "oscillation_timeout";

/// Core-required blackboard entries (validateRequiredBlackboardEntries
/// scans them once after createTree). Recovery role ids
/// (k_clear_costmap_id/k_spin_id/k_wait_id/k_backup_id) are NOT core: they
/// are declared and provided by the BT extension that references them
/// (BehaviorTreeExtension::requiredBlackboardEntries/provideBlackboardEntries)
/// -- the BT only orchestrates recoveries and never decides what they do.
inline const std::vector<const char*>& requiredBlackboardEntries() {
  static const std::vector<const char*> k_required = {
    k_goal, k_raw_path, k_smoothed_path, k_planner_frequency, k_smoother_frequency,
    k_controller_frequency,
    k_session_id, k_raw_path_revision,
    k_failure_context, k_goal_revision, k_nav_retries,
  };
  return k_required;
}

/// @brief The failure_context port pair shared by every failure-producing
/// node and every RecoveryNode: the structured context travels through the
/// declared port binding ({failure_context} in the tree XML) instead of an
/// implicit blackboard slot, so the tree build validates the wiring.
inline BT::PortsList failureContextPorts() {
  return {
      BT::OutputPort<FailureContext>(k_failure_context,
                                     "structured context written before "
                                     "FAILURE"),
  };
}

/// Verify all protocol-required blackboard entries exist; throw immediately on any missing one.
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
