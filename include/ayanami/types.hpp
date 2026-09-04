#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/utility/config.hpp"

namespace ayanami {

/// @brief 插件注入上下文: 逻辑名 + 插件自身配置文件 (已解析)。
///
/// 插件参数统一从这里读取, 插件不知道全局配置文件布局/索引结构。
/// config 即 config_{family} 指向的模块 json (含插件描述段与参数段)。
struct PluginContext {
  std::string name;
  Config config;
};

/// Business outcome used by plugins and BT decisions.
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

  // Transitional spellings for the existing public API.
  None = NONE,
  Canceled = CANCELED,
  Timeout = TIMEOUT,
  InvalidInput = INVALID_INPUT,
  TransformUnavailable = TRANSFORM_UNAVAILABLE,
  MapUnavailable = MAP_UNAVAILABLE,
  MapStale = MAP_STALE,
  StartInvalid = START_INVALID,
  GoalInvalid = GOAL_INVALID,
  PathNotFound = PATH_NOT_FOUND,
  PathInvalid = PATH_INVALID,
  SmoothingFailed = SMOOTHING_FAILED,
  NoValidCommand = NO_VALID_COMMAND,
  NoProgress = NO_PROGRESS,
  GoalNotReached = GOAL_NOT_REACHED,
  RecoveryFailed = RECOVERY_FAILED,
  InternalError = INTERNAL_ERROR,
  StaleResult = STALE_RESULT,
  ResourceBusy = RESOURCE_BUSY,
};

template <typename DataT>
struct AbstractResultT {
  bool ok{false};
  DataT data{};
  ErrorCode error{ErrorCode::INTERNAL_ERROR};
  std::string message;

  static AbstractResultT success(const DataT& value) {
    return {true, value, ErrorCode::NONE, {}};
  }

  static AbstractResultT success(DataT&& value) {
    return {true, std::move(value), ErrorCode::NONE, {}};
  }

  static AbstractResultT failure(ErrorCode code, const std::string& text) {
    return {false, {}, code, text};
  }

  explicit operator bool() const noexcept { return ok; }
};

using PlannerResultT = AbstractResultT<nav_msgs::Path>;
using SmootherResultT = AbstractResultT<nav_msgs::Path>;

struct ControllerCommand {
  geometry_msgs::TwistStamped velocity;
};
using ControllerResultT = AbstractResultT<ControllerCommand>;

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
using RecoveryResultT = AbstractResultT<RecoveryCycleData>;

}  // namespace ayanami
