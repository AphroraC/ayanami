#pragma once

#include <string>

#include "ayanami/messages/actions.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/messages/swerve.hpp"
#include "ayanami/utility/callback_slot.hpp"

namespace ayanami {
namespace interfaces {

/**
 * @brief Aggregate of global side-path callback slots.
 *
 * The slot objects' static members are defined exactly once in libayanami's
 * src/interfaces/callbacks.cpp (not inline, key discipline): otherwise each
 * .so would hold its own instance, and callbacks added by modules and
 * emitted by the core would land on different objects, silently failing.
 *
 * Division of labor: the mainline control flow goes through BT nodes
 * calling NavigationServerBase's TaskHandle directly and does not pass
 * here; this group of slots serves side paths (ROS extension /
 * visualization / fallback / external monitoring) subscriptions.
 *
 * Input/output discipline: these global slots carry only **output/
 * observation** traffic (goal/feedback/result, velocity command, costmap
 * visualization). Inputs such as obstacle observations (pointcloud/
 * laserscan) do not go through global slots -- the ROS side calls
 * AbstractNavigation::setPointcloud / setLaserScan directly ->
 * CostmapNavigationServer::handleObstacle* to stamp the map (handed
 * straight to the host).
 */
struct NavigationCallbacks {
  // ---- Four-component action streams (goal / feedback / result) ----
  static CallbackSlot<void(const ayanami::actions::GetPathGoal&)> on_get_path_goal;
  static CallbackSlot<void(const ayanami::actions::GetPathFeedback&)> on_get_path_feedback;
  static CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::GetPathResult>&)> on_get_path_result;
  static CallbackSlot<void(const ayanami::actions::SmoothPathGoal&)> on_smooth_path_goal;
  static CallbackSlot<void(const ayanami::actions::SmoothPathFeedback&)> on_smooth_path_feedback;
  static CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::SmoothPathResult>&)> on_smooth_path_result;
  static CallbackSlot<void(const ayanami::actions::ExePathGoal&)> on_exe_path_goal;
  static CallbackSlot<void(const ayanami::actions::ExePathFeedback&)> on_exe_path_feedback;
  static CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::ExePathResult>&)> on_exe_path_result;
  static CallbackSlot<void(const ayanami::actions::RecoveryGoal&)> on_recovery_goal;
  static CallbackSlot<void(const ayanami::actions::RecoveryFeedback&)> on_recovery_feedback;
  static CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::RecoveryResult>&)> on_recovery_result;
  // ---- State / output / observation (the remaining slots;
  // on_robot_state_updated had no production emit point and was removed) ----
  static CallbackSlot<void(const ayanami::geometry_msgs::TwistStamped&)> on_velocity_command;
  /// @brief Derived swerve wheel-level view of the latest velocity command
  /// (rigid-body inverse of the twist; fl, fr, rl, rr). Telemetry for a
  /// future direct-chassis bridge / diagnostics -- the twist channel stays
  /// authoritative.
  static CallbackSlot<void(const ayanami::messages::SwerveCommandsStamped&)> on_swerve_command;
  // ---- Costmap layer streams (one slot per layer artifact; the sole map
  // writer emits unconditionally whenever that artifact's content changes --
  // generation-gated, so an unchanged map stays silent and no rate parameter
  // exists anywhere). Receivers hold their own cache: a global-map artifact
  // may be emitted only once (static global policy), the receiver republishes.
  static CallbackSlot<void(const ayanami::nav_msgs::OccupancyGrid&)> on_global_costmap_static;
  static CallbackSlot<void(const ayanami::nav_msgs::OccupancyGrid&)> on_global_costmap_inflation;
  /// @brief The local map's raw obstacle marks (SimpleObstacleLayer's
  /// presence grid, NO inflation): dynamic obstacles only, no decayed halo.
  static CallbackSlot<void(const ayanami::nav_msgs::OccupancyGrid&)> on_local_costmap_obstacle;
  static CallbackSlot<void(const ayanami::nav_msgs::OccupancyGrid&)> on_local_costmap_inflation;
  /// @brief The rosemap family's single field artifact (binarized occupancy
  /// + EDT source grid), emitted by the sole rosemap writer whenever the
  /// snapshot sequence advances -- same generation-gated unconditional
  /// semantics as the costmap layer slots.
  static CallbackSlot<void(const ayanami::nav_msgs::OccupancyGrid&)> on_rosemap_field;
  // ---- session-level navigation terminal outcome (once per tree life) ----
  static CallbackSlot<void(const ayanami::actions::NavigationResult&)> on_navigation_result;
};

} // namespace interfaces
} // namespace ayanami
