// dwb_types.hpp — DWB 专用 2D 类型 (nav_2d_msgs / dwb_msgs / geometry_msgs
// 2D 消息的纯 C++ 等价物, 迁移自 nav2_dwb_controller, 零 ROS 依赖)。
//
// 迁移说明: 原 nav2 用 geometry_msgs::Pose2D / nav_2d_msgs::Twist2D /
// Path2D / dwb_msgs::Trajectory2D 等消息; 本模块内以原生 struct 等价替换,
// 字段与语义逐一对应。time_offsets 由 builtin_interfaces::Duration 换为
// double 秒。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "ayanami/messages/timestamp.hpp"

namespace geometry_msgs {

/// @brief 等价 geometry_msgs::msg::Pose2D (x, y, theta)。
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

/// @brief 等价 geometry_msgs::msg::Point (footprint 用, 仅 2D 语义)。
struct Point {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

}  // namespace geometry_msgs

namespace nav_2d_msgs {

/// @brief 等价 nav_2d_msgs::msg::Twist2D (x, y, theta)。
struct Twist2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

/// @brief 等价 nav_2d_msgs::msg::Pose2DStamped。
struct Pose2DStamped {
  std::string frame_id;
  ayanami::Timestamp stamp;
  geometry_msgs::Pose2D pose;
};

/// @brief 等价 nav_2d_msgs::msg::Twist2DStamped。
struct Twist2DStamped {
  std::string frame_id;
  ayanami::Timestamp stamp;
  Twist2D velocity;
};

/// @brief 等价 nav_2d_msgs::msg::Path2D。
struct Path2D {
  std::string frame_id;
  ayanami::Timestamp stamp;
  std::vector<geometry_msgs::Pose2D> poses;
};

}  // namespace nav_2d_msgs

namespace dwb_msgs {

/// @brief 等价 dwb_msgs::msg::Trajectory2D (time_offsets 用 double 秒)。
struct Trajectory2D {
  nav_2d_msgs::Twist2D velocity;
  std::vector<double> time_offsets;
  std::vector<geometry_msgs::Pose2D> poses;
};

/// @brief 等价 dwb_msgs::msg::CriticScore。
struct CriticScore {
  std::string name;
  double raw_score = 0.0;
  double scale = 0.0;
};

/// @brief 等价 dwb_msgs::msg::TrajectoryScore。
struct TrajectoryScore {
  Trajectory2D traj;
  std::vector<CriticScore> scores;
  double total = 0.0;
};

/// @brief 等价 dwb_msgs::msg::LocalPlanEvaluation (调试数据)。
struct LocalPlanEvaluation {
  std::string frame_id;
  std::vector<TrajectoryScore> twists;
  std::uint16_t best_index = 0;
  std::uint16_t worst_index = 0;
};

}  // namespace dwb_msgs
