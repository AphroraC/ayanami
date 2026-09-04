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

// 统一失败码: 四类 Result 都用 `error` (ErrorCode) 传达成败 (NONE=成功),
// message 仅诊断 (BT 不解析)。每个 Goal/Result 带 session_id / plan_version
// 用于 BT 节点 onRunning 竞态校验。

struct GetPathGoal {
  ayanami::geometry_msgs::PoseStamped start;
  ayanami::geometry_msgs::PoseStamped goal;
  double tolerance = 0.1;
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

struct GetPathFeedback {
  std::string message;
  using Ptr = std::shared_ptr<GetPathFeedback>;
  using ConstPtr = std::shared_ptr<const GetPathFeedback>;
};

struct SmoothPathGoal {
  ayanami::nav_msgs::Path path;
  double max_duration = 2.0;
  bool check_for_collision = false;
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

struct SmoothPathFeedback {
  std::string message;
  using Ptr = std::shared_ptr<SmoothPathFeedback>;
  using ConstPtr = std::shared_ptr<const SmoothPathFeedback>;
};

struct ExePathGoal {
  ayanami::nav_msgs::Path path;
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

struct ExePathFeedback {
  std::string message;
  using Ptr = std::shared_ptr<ExePathFeedback>;
  using ConstPtr = std::shared_ptr<const ExePathFeedback>;
};

struct RecoveryGoal {
  std::string recovery_id;
  /// @brief 触发本次恢复动作的失败上下文 (BT Recovery 节点从黑板注入)。
  ayanami::FailureContext cause;
  /// @brief 按次覆盖参数 (nullopt 字段 = 插件 configure 期默认值)。
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

struct RecoveryFeedback {
  std::string message;
  using Ptr = std::shared_ptr<RecoveryFeedback>;
  using ConstPtr = std::shared_ptr<const RecoveryFeedback>;
};

// 目标状态 (spec §6.2 用 GoalStatus::SUCCEEDED 做 BT 结果校验)。
enum class GoalStatus : uint8_t {
  PENDING = 0,
  ACTIVE = 1,
  SUCCEEDED = 2,
  CANCELED = 3,
  ABORTED = 4,
};

/**
 * @brief 带状态的 action 结果聚合 (spec §3.3 槽签名用)。
 *
 * TaskHandle 完成时用 GoalResult<Result> 交付: status 描述整体目标状态,
 * result 携带各 action 族的 Result (含 failure/session_id/plan_version)。
 */
template <typename Result>
struct GoalResult {
  GoalStatus status = GoalStatus::PENDING;
  Result result;
  using Ptr = std::shared_ptr<GoalResult>;
  using ConstPtr = std::shared_ptr<const GoalResult>;
};

} // namespace actions
} // namespace ayanami
