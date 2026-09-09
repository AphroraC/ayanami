#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/messages/recovery_params.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace ayanami {
namespace actions {

// Unified failure codes: all four Result types use `error` (ErrorCode) to
// convey success/failure (NONE=success); message is diagnostic only (the BT
// does not parse it). Every Goal/Result carries session_id / plan_version
// for BT nodes onRunning race checks.
//
// Plugin selection: the first three families support multiple configured
// plugins (nav2 semantics). The *_id goal field selects the plugin at
// request time; an empty id falls back to the family default (the first
// configured plugin), which is also what single-plugin configs use.

struct GetPathGoal {
  ayanami::geometry_msgs::PoseStamped start;
  ayanami::geometry_msgs::PoseStamped goal;
  double tolerance = 0.1;
  /// @brief Planner plugin id (nav2 goal-level selection; empty = default).
  std::string planner_id;
  uint64_t session_id = 0;
  uint64_t plan_version = 0;
  using Ptr = std::shared_ptr<GetPathGoal>;
  using ConstPtr = std::shared_ptr<const GetPathGoal>;
};

struct GetPathResult {
  ErrorCode error = ErrorCode::NONE;
  std::string message;
  ayanami::nav_msgs::Path path;
  double cost = 0.0;
  uint64_t session_id = 0;
  uint64_t plan_version = 0;
  using Ptr = std::shared_ptr<GetPathResult>;
  using ConstPtr = std::shared_ptr<const GetPathResult>;
};

/// @brief Planning feedback (nav2 has no ComputePathToPose feedback; ayanami
/// carries the structured stage message only, via the on_get_path_feedback
/// slot).
struct GetPathFeedback {
  std::string message;
  using Ptr = std::shared_ptr<GetPathFeedback>;
  using ConstPtr = std::shared_ptr<const GetPathFeedback>;
};

struct SmoothPathGoal {
  ayanami::nav_msgs::Path path;
  double max_duration = 2.0;
  /// @brief Smoother plugin id (nav2 goal-level selection; empty = default).
  std::string smoother_id;
  uint64_t session_id = 0;
  uint64_t plan_version = 0;
  using Ptr = std::shared_ptr<SmoothPathGoal>;
  using ConstPtr = std::shared_ptr<const SmoothPathGoal>;
};

struct SmoothPathResult {
  ErrorCode error = ErrorCode::NONE;
  std::string message;
  ayanami::nav_msgs::Path smoothed_path;
  uint64_t session_id = 0;
  uint64_t plan_version = 0;
  using Ptr = std::shared_ptr<SmoothPathResult>;
  using ConstPtr = std::shared_ptr<const SmoothPathResult>;
};

/// @brief Smoothing feedback (nav2 has no SmoothPath feedback; stage message
/// only, via the on_smooth_path_feedback slot).
struct SmoothPathFeedback {
  std::string message;
  using Ptr = std::shared_ptr<SmoothPathFeedback>;
  using ConstPtr = std::shared_ptr<const SmoothPathFeedback>;
};

struct ExePathGoal {
  ayanami::nav_msgs::Path path;
  /// @brief Controller plugin id (nav2 goal-level selection; empty = default).
  std::string controller_id;
  uint64_t session_id = 0;
  uint64_t plan_version = 0;
  using Ptr = std::shared_ptr<ExePathGoal>;
  using ConstPtr = std::shared_ptr<const ExePathGoal>;
};

struct ExePathResult {
  ErrorCode error = ErrorCode::NONE;
  std::string message;
  bool reached = false;
  uint64_t session_id = 0;
  uint64_t plan_version = 0;
  using Ptr = std::shared_ptr<ExePathResult>;
  using ConstPtr = std::shared_ptr<const ExePathResult>;
};

/// @brief Control feedback (nav2 FollowPath semantics): current speed,
/// remaining path length from the nearest path point, and the estimated time
/// to goal at the running average speed. Emitted by the controller execution
/// every control cycle through the on_exe_path_feedback slot.
struct ExePathFeedback {
  double speed = 0.0;
  double distance_remaining = 0.0;
  double estimated_time_remaining = 0.0;
  std::string message;
  using Ptr = std::shared_ptr<ExePathFeedback>;
  using ConstPtr = std::shared_ptr<const ExePathFeedback>;
};

struct RecoveryGoal {
  std::string recovery_id;
  /// @brief The failure context that triggered this recovery action (injected from the blackboard by the BT Recovery node).
  ayanami::FailureContext cause;
  /// @brief Per-run override params (nullopt field = the plugin configure-time default).
  ayanami::RecoveryBehaviorParams behavior;
  uint64_t session_id = 0;
  uint64_t plan_version = 0;
  using Ptr = std::shared_ptr<RecoveryGoal>;
  using ConstPtr = std::shared_ptr<const RecoveryGoal>;
};

struct RecoveryResult {
  ErrorCode error = ErrorCode::NONE;
  std::string message;
  uint64_t session_id = 0;
  uint64_t plan_version = 0;
  using Ptr = std::shared_ptr<RecoveryResult>;
  using ConstPtr = std::shared_ptr<const RecoveryResult>;
};

/// @brief Recovery feedback (nav2 TimedBehavior has none; stage message
/// only, via the on_recovery_feedback slot).
struct RecoveryFeedback {
  std::string message;
  using Ptr = std::shared_ptr<RecoveryFeedback>;
  using ConstPtr = std::shared_ptr<const RecoveryFeedback>;
};

// goal status (GoalStatus::SUCCEEDED is used for BT result validation).
enum class GoalStatus : uint8_t {
  PENDING = 0,
  ACTIVE = 1,
  SUCCEEDED = 2,
  CANCELED = 3,
  ABORTED = 4,
};

/**
 * @brief Action result aggregate with status (used by slot signatures).
 *
 * On TaskHandle completion, GoalResult<Result> is delivered: status
 * describes the overall goal state, and result carries each action family Result
 * (including failure/session_id/plan_version).
 */
template <typename Result>
struct GoalResult {
  GoalStatus status = GoalStatus::PENDING;
  Result result;
  using Ptr = std::shared_ptr<GoalResult>;
  using ConstPtr = std::shared_ptr<const GoalResult>;
};

/// @brief Session-level navigation terminal outcome (observability exit).
/// Emitted exactly once per tree life by the pilot: SUCCEEDED / ABORTED
/// (tree FAILURE, context carries the structured failure) / CANCELED
/// (halt while a navigation was in flight). status never takes the
/// transient values (PENDING/ACTIVE).
struct NavigationResult {
  GoalStatus status = GoalStatus::ABORTED;
  FailureContext context;
  using Ptr = std::shared_ptr<NavigationResult>;
  using ConstPtr = std::shared_ptr<const NavigationResult>;
};

} // namespace actions
} // namespace ayanami
