#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/utility/config.hpp"

namespace ayanami {

/// @brief Module injection context: logical name + the module's own config
/// file (already parsed).
///
/// Module parameters are read uniformly from here; modules do not know the
/// global config file layout/index structure.
/// config is the module json pointed to by config_{family} (containing the
/// module description and parameter sections).
struct ModuleContext {
  std::string name;
  Config config;
};

/// Business outcome used by modules and BT decisions.
enum class ErrorCode : std::uint8_t {
  NONE = 0,
  CANCELED,
  TIMEOUT,
  INVALID_INPUT,
  TRANSFORM_UNAVAILABLE,
  MAP_UNAVAILABLE,
  MAP_STALE,
  START_INVALID,
  GOAL_INVALID,
  PATH_NOT_FOUND,
  PATH_INVALID,
  SMOOTHING_FAILED,
  NO_VALID_COMMAND,
  NO_PROGRESS,
  GOAL_NOT_REACHED,
  RECOVERY_FAILED,
  INTERNAL_ERROR,
  STALE_RESULT,
  RESOURCE_BUSY,
};

/// @brief Unified result for all four module families (planner / smoother /
/// controller / recovery). The module surface returns ResultT<void>: computed
/// payloads (path / twist / cycle data) travel through caller-owned
/// out-parameters (or in-place inputs) so a single result type serves every
/// module without per-family aliases, and call sites may reuse buffers across
/// cycles. The primary template stays available for future payload-carrying
/// results; do not introduce one for the existing four families.
template <typename DataT>
struct ResultT {
  bool ok{false};
  DataT data{};
  ErrorCode error{ErrorCode::INTERNAL_ERROR};
  std::string message;

  // Injected class name: spelling ResultT<void> here would name the
  // specialization below and instantiate it before its declaration.
  static ResultT success(const DataT& value) {
    return {true, value, ErrorCode::NONE, {}};
  }

  static ResultT success(DataT&& value) {
    return {true, std::move(value), ErrorCode::NONE, {}};
  }

  static ResultT failure(ErrorCode code, const std::string& text) {
    return {false, DataT{}, code, text};
  }

  explicit operator bool() const noexcept { return ok; }
};

/// @brief Payload-free specialization: the result type every algorithm module
/// returns. ok=true means the out-parameters are valid; failure carries the
/// ErrorCode + diagnostic message (BT decisions read the code, never the
/// message).
template <>
struct ResultT<void> {
  bool ok{false};
  ErrorCode error{ErrorCode::INTERNAL_ERROR};
  std::string message;

  static ResultT<void> success() { return {true, ErrorCode::NONE, {}}; }

  static ResultT<void> failure(ErrorCode code, const std::string& text) {
    return {false, code, text};
  }

  explicit operator bool() const noexcept { return ok; }
};

/// @brief Pose tolerance (semantics equivalent to geometry_msgs/
/// PoseWithCovariance).
///
/// position: xy planar position tolerance (m); angle: yaw angle tolerance (rad).
/// Lives here with the other shared value types because two components need
/// it: the goal checker owns the arrival contract, and the controller must
/// converge into that same contract.
struct PoseTolerance {
  double position = 0.10;
  double angle = 0.20;
};

/// @brief Velocity tolerance (body frame, stop-decision boundary).
///
/// linear: linear velocity tolerance (m/s); angular: angular velocity
/// tolerance (rad/s).
struct TwistTolerance {
  double linear = 0.10;
  double angular = 0.10;
};

/// @brief Combined goal-check tolerances (getTolerances return value).
struct GoalCheckerTolerances {
  PoseTolerance pose;
  TwistTolerance velocity;
};

enum class RecoveryStatus : std::uint8_t {
  RUNNING,
  SUCCESS,
  FAILED,
  CANCELED,
};

struct RecoveryCycleData {
  RecoveryStatus status{RecoveryStatus::RUNNING};
  std::optional<geometry_msgs::TwistStamped> velocity;
};

}  // namespace ayanami
