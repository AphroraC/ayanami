#pragma once

#include <cstdint>
#include <string>

#include "ayanami/types.hpp"

namespace ayanami {

/**
 * @brief Framework-wide strongly-typed failure codes (replacing the
 * planner/controller/smoother/recovery 50/100/200/300 local code ranges
 * and the uint32 outcome).
 *
 * Discipline:
 * - The four plugin interfaces (AbstractPlanner/Smoother/Controller/
 *   Recovery) always return this type;
 * - Each action Result uses the unified field `failure`; None means success
 *   (corresponding
 *   GoalStatus::SUCCEEDED);
 * - GoalStatus is retained; message is diagnostic only -- BT/hosts decide on failure only and never parse message.
 */
// Kept only as a source-compatibility alias for message adapters and old
// plugins. BT and all new core APIs use ErrorCode directly.
using FailureCode = ErrorCode;


/**
 * @brief The processing stage where the failure occurred (used by BT action nodes writing FailureContext).
 */
enum class FailureStage : uint8_t {
  Planner = 0,
  Smoother = 1,
  Controller = 2,
  Recovery = 3,
  Navigation = 4,
  Unknown = 5,
};

/**
 * @brief Structured failure context (key discipline extension).
 *
 * Every BT action node (ComputePathToPose / SmoothPath / FollowPath /
 * Recovery) writes the precise failure context to the blackboard
 * `failure_context` (blackboard_protocol.hpp) before returning FAILURE.
 * RecoveryGoal.cause carries this context so the recovery behavior can
 * identify the failure that triggered it (plugin id / failure code /
 * owning session version).
 *
 * The blackboard entry is replaced wholesale (not incrementally merged):
 * the node's latest failure overwrites the previous one.
 */
struct FailureContext {
  FailureStage stage = FailureStage::Unknown;
  ErrorCode code = ErrorCode::NONE;
  std::string plugin_id;   ///< Logical name of the plugin that triggered the failure
  std::string message;     ///< Diagnostic message (for logs only; BT/hosts do not parse it)
  uint64_t session_id = 0;  ///< Session the failure belongs to
  uint64_t plan_version = 0;  ///< Plan version the failure belongs to
};

}  // namespace ayanami
