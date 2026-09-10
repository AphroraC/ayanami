#pragma once

#include <string>

#include "ayanami/messages/actions.hpp"
#include "ayanami/messages/nav_msgs.hpp"
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
  static CallbackSlot<void(const std::string&, const ayanami::nav_msgs::OccupancyGrid&)> on_costmap_visualization;
  // ---- session-level navigation terminal outcome (once per tree life) ----
  static CallbackSlot<void(const ayanami::actions::NavigationResult&)> on_navigation_result;
};

} // namespace interfaces
} // namespace ayanami
