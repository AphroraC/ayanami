#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

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
inline bool goalEquals(const geometry_msgs::PoseStamped& lhs, const geometry_msgs::PoseStamped& rhs) {
  if (lhs.header.stamp.count() != rhs.header.stamp.count()) {
    return false;
  }
  if (lhs.header.frame_id != rhs.header.frame_id) {
    return false;
  }
  // Absolute tolerance (see poseClose): the relative isApprox never matched
  // near the frame origin.
  return ayanami::geometry_msgs::poseClose(lhs.pose, rhs.pose);
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
// Producer-scoped revision of the raw path: ComputePathToPose advances it
// when a new raw path lands. For the smoother branch it is the
// RE-INITIALIZATION signal: iterative refinement keeps taking the current
// {smoothed_path} as its input, and only a changed revision restarts from
// {raw_path} (a new planned route must not be refined from the old route's
// smoothed path).
inline constexpr const char* k_raw_path_revision = "raw_path_revision";
inline constexpr const char* k_goal_revision = "goal_revision";
inline constexpr const char* k_failure_context = "failure_context";
// Outer RecoveryNode retry budget, injected from navigation.max_nav_retries
// (config owns the policy; the tree XML only binds the port to this key).
inline constexpr const char* k_nav_retries = "nav_retries";
// BT-level deadline budgets for the planner/smoother branches (MS;
// navigation.planner_timeout/smoother_timeout seconds, 0 = unlimited).
// Written by the pilot like the oscillation keys: optional protocol keys
// whose absence degrades to the port defaults (unlimited), never a build
// failure.
inline constexpr const char* k_planner_timeout = "planner_timeout";
inline constexpr const char* k_smoother_timeout = "smoother_timeout";
// Outer RecoveryNode same-cause pruning budget
// (navigation.max_same_cause, 0 = off): a failure code that repeats this
// many consecutive times is treated as unrecoverable for this attempt.
inline constexpr const char* k_max_same_cause = "max_same_cause";
// Navigation-level oscillation trigger parameters (MBF semantics),
// injected from navigation.oscillation_{distance,timeout}.
inline constexpr const char* k_oscillation_distance = "oscillation_distance";
inline constexpr const char* k_oscillation_timeout = "oscillation_timeout";
// Tree tick rate (navigation.tick_frequency), written by the pilot so nodes
// that size time-windowed buffers (IsRobotProgressing) never read Config on
// the tick path. Optional: readers fall back to the 100 Hz default when the
// entry is absent (hand-built test trees).
inline constexpr const char* k_tick_frequency = "tick_frequency";

/// Core-required blackboard entries (validateRequiredBlackboardEntries
/// scans them once after createTree). Recovery role ids
/// (k_clear_costmap_id/k_spin_id/k_wait_id/k_backup_id) are NOT core: they
/// are declared and provided by the BT extension that references them
/// (BehaviorTreeExtension::requiredBlackboardEntries/provideBlackboardEntries)
/// -- the BT only orchestrates recoveries and never decides what they do.
inline const std::vector<const char*>& requiredBlackboardEntries() {
  static const std::vector<const char*> k_required = {
    k_goal,
    k_raw_path,
    k_smoothed_path,
    k_planner_frequency,
    k_smoother_frequency,
    k_controller_frequency,
    k_session_id,
    k_raw_path_revision,
    k_failure_context,
    k_goal_revision,
    k_nav_retries,
  };
  return k_required;
}

/// @brief Per-attempt blackboard reset for a NEW navigation attempt on the
/// RESIDENT tree (pilot-side; nav2 parity where Navigator::
/// initializeGoalPose rewrites the task-level keys per goal and
/// ComputePathToPoseAction::halt empties the path artifact so a mis-timed
/// reader fails instead of navigating the previous route).
///
/// The full per-attempt key set -- {goal}, {raw_path}, {smoothed_path},
/// {raw_path_revision}, {session_id}, {goal_revision}, {failure_context} --
/// is rewritten by the pilot's applyPendingGoal on EVERY consumed goal (new
/// attempt or in-flight revision alike); this helper clears the two path
/// ARTIFACTS, which are the only keys a finished attempt leaves behind that
/// a new attempt must never observe.
inline void resetAttemptBlackboard(const BT::Blackboard::Ptr& blackboard) {
  blackboard->set<nav_msgs::Path>(k_raw_path, nav_msgs::Path{});
  blackboard->set<nav_msgs::Path>(k_smoothed_path, nav_msgs::Path{});
}

/// @brief The failure_context port pair shared by every failure-producing
/// node and every RecoveryNode: the structured context travels through the
/// declared port binding ({failure_context} in the tree XML) instead of an
/// implicit blackboard slot, so the tree build validates the wiring.
inline BT::PortsList failureContextPorts() {
  return {
    BT::OutputPort<FailureContext>(
      k_failure_context,
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
      throw std::runtime_error(std::string("blackboard_protocol: required entry missing: ") + key);
    }
  }
}

}  // namespace behavior_tree
}  // namespace ayanami
