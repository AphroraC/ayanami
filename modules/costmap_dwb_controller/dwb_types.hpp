// dwb_types.hpp -- DWB-specific 2D types (pure C++ equivalents of the
// nav_2d_msgs / dwb_msgs / geometry_msgs 2D messages, ported from nav2_dwb_controller, zero ROS dependency).
//
// Porting notes: the original nav2 uses geometry_msgs::Pose2D / nav_2d_msgs::Twist2D /
// Path2D / dwb_msgs::Trajectory2D and similar messages; within this module they are replaced by equivalent native structs,
// with fields and semantics matching one-to-one. time_offsets changes from builtin_interfaces::Duration to
// double seconds.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "ayanami/messages/timestamp.hpp"

namespace geometry_msgs {

/// @brief Equivalent of geometry_msgs::msg::Pose2D (x, y, theta).
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

/// @brief Equivalent of geometry_msgs::msg::Point (used for footprints, 2D semantics only).
struct Point {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

}  // namespace geometry_msgs

namespace nav_2d_msgs {

/// @brief Equivalent of nav_2d_msgs::msg::Twist2D (x, y, theta).
struct Twist2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

/// @brief Equivalent of nav_2d_msgs::msg::Pose2DStamped.
struct Pose2DStamped {
  std::string frame_id;
  ayanami::Timestamp stamp;
  geometry_msgs::Pose2D pose;
};

/// @brief Equivalent of nav_2d_msgs::msg::Twist2DStamped.
struct Twist2DStamped {
  std::string frame_id;
  ayanami::Timestamp stamp;
  Twist2D velocity;
};

/// @brief Equivalent of nav_2d_msgs::msg::Path2D.
struct Path2D {
  std::string frame_id;
  ayanami::Timestamp stamp;
  std::vector<geometry_msgs::Pose2D> poses;
};

}  // namespace nav_2d_msgs

namespace dwb_msgs {

/// @brief Equivalent of dwb_msgs::msg::Trajectory2D (time_offsets as double seconds).
struct Trajectory2D {
  nav_2d_msgs::Twist2D velocity;
  std::vector<double> time_offsets;
  std::vector<geometry_msgs::Pose2D> poses;
};

/// @brief Equivalent of dwb_msgs::msg::CriticScore.
struct CriticScore {
  std::string name;
  double raw_score = 0.0;
  double scale = 0.0;
};

/// @brief Equivalent of dwb_msgs::msg::TrajectoryScore.
struct TrajectoryScore {
  Trajectory2D traj;
  std::vector<CriticScore> scores;
  double total = 0.0;
};

/// @brief Equivalent of dwb_msgs::msg::LocalPlanEvaluation (debug data).
struct LocalPlanEvaluation {
  std::string frame_id;
  std::vector<TrajectoryScore> twists;
  std::uint16_t best_index = 0;
  std::uint16_t worst_index = 0;
};

}  // namespace dwb_msgs
