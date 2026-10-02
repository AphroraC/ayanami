#include "ayanami/interfaces/callbacks.hpp"

namespace ayanami {
namespace interfaces {

// Slot object definitions -- exactly one copy inside libayanami (key
// discipline).
// inline static is not used: otherwise each .so would hold its own instance,
// and callbacks added by modules and emitted by the core would land on
// different objects, silently failing across .so boundaries.

CallbackSlot<void(const ayanami::actions::GetPathGoal&)> NavigationCallbacks::on_get_path_goal;
CallbackSlot<void(const ayanami::actions::GetPathFeedback&)> NavigationCallbacks::on_get_path_feedback;
CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::GetPathResult>&)> NavigationCallbacks::on_get_path_result;
CallbackSlot<void(const ayanami::actions::SmoothPathFeedback&)> NavigationCallbacks::on_smooth_path_feedback;
CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::SmoothPathResult>&)> NavigationCallbacks::on_smooth_path_result;
CallbackSlot<void(const ayanami::actions::ExePathFeedback&)> NavigationCallbacks::on_exe_path_feedback;
CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::ExePathResult>&)> NavigationCallbacks::on_exe_path_result;
CallbackSlot<void(const ayanami::actions::RecoveryGoal&)> NavigationCallbacks::on_recovery_goal;
CallbackSlot<void(const ayanami::actions::RecoveryFeedback&)> NavigationCallbacks::on_recovery_feedback;
CallbackSlot<void(const ayanami::actions::GoalResult<ayanami::actions::RecoveryResult>&)> NavigationCallbacks::on_recovery_result;
CallbackSlot<void(const ayanami::geometry_msgs::TwistStamped&)> NavigationCallbacks::on_velocity_command;
CallbackSlot<void(const ayanami::nav_msgs::OccupancyGrid&)> NavigationCallbacks::on_global_costmap_static;
CallbackSlot<void(const ayanami::nav_msgs::OccupancyGrid&)> NavigationCallbacks::on_global_costmap_inflation;
CallbackSlot<void(const ayanami::nav_msgs::OccupancyGrid&)> NavigationCallbacks::on_local_costmap_obstacle;
CallbackSlot<void(const ayanami::nav_msgs::OccupancyGrid&)> NavigationCallbacks::on_local_costmap_inflation;
CallbackSlot<void(const ayanami::nav_msgs::EsdfGrid&)>
    NavigationCallbacks::on_esdfmap_field;
CallbackSlot<void(const ayanami::actions::NavigationResult&)>
    NavigationCallbacks::on_navigation_result;

} // namespace interfaces
} // namespace ayanami
