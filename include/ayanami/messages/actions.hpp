#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace ayanami {
namespace actions {

// 错误码段 (spec §5.1): 0=SUCCESS; Planner 50+; Controller 100+; Smoother 200+;
// Recovery 300+。Result::outcome 存实际返回码，且每个 Goal/Result 带
// session_id / plan_version 用于 BT 节点 onRunning 竞态校验。

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
  // planner 错误段 50+ (NO_PATH_FOUND=58 见 spec §5.1)
  static constexpr uint32_t SUCCESS = 0;
  static constexpr uint32_t INVALID_START = 51;
  static constexpr uint32_t INVALID_GOAL = 52;
  static constexpr uint32_t TF_ERROR = 53;
  static constexpr uint32_t OUT_OF_MAP = 54;
  static constexpr uint32_t NO_PATH_FOUND = 58;
  static constexpr uint32_t INTERNAL_ERROR = 59;

  uint32_t outcome = SUCCESS;
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
  // smoother 错误段 200+
  static constexpr uint32_t SUCCESS = 0;
  static constexpr uint32_t EMPTY_PATH = 201;
  static constexpr uint32_t NO_SMOOTHED_PATH = 202;
  static constexpr uint32_t TIMEOUT = 203;
  static constexpr uint32_t INTERNAL_ERROR = 299;

  uint32_t outcome = SUCCESS;
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
  // controller 错误段 100+
  static constexpr uint32_t SUCCESS = 0;
  static constexpr uint32_t NO_VALID_COMMAND = 101;
  static constexpr uint32_t PAT_EXCEEDED = 102;
  static constexpr uint32_t NOT_REACHED = 103;
  static constexpr uint32_t INTERNAL_ERROR = 199;

  uint32_t outcome = SUCCESS;
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
  uint64_t session_id = 0;
  uint64_t plan_version = 0;
  using Ptr = std::shared_ptr<RecoveryGoal>;
  using ConstPtr = std::shared_ptr<const RecoveryGoal>;
};

struct RecoveryResult {
  // recovery 错误段 300+
  static constexpr uint32_t SUCCESS = 0;
  static constexpr uint32_t INTERNAL_ERROR = 300;
  static constexpr uint32_t CANCELED = 301;
  static constexpr uint32_t PAT_EXCEEDED = 302;

  uint32_t outcome = SUCCESS;
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

} // namespace actions
} // namespace ayanami
