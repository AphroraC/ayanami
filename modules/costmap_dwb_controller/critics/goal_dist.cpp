// goal_dist.cpp — 迁移自 dwb_critics/src/goal_dist.cpp。
#include "goal_dist.hpp"

#include <vector>

#include <costmap_2d/cost_values.hpp>

#include "../path_ops.hpp"

#include <spdlog/spdlog.h>

namespace dwb_critics {
bool GoalDistCritic::prepare(const geometry_msgs::Pose2D&,
                             const nav_2d_msgs::Twist2D&,
                             const geometry_msgs::Pose2D&,
                             const nav_2d_msgs::Path2D& global_plan) {
  reset();

  unsigned int local_goal_x, local_goal_y;
  if (!getLastPoseOnCostmap(global_plan, local_goal_x, local_goal_y)) {
    return false;
  }

  // Enqueue just the last pose
  int index = costmap_->getIndex(local_goal_x, local_goal_y);
  cell_values_[index] = 0.0;
  queue_->enqueueCell(local_goal_x, local_goal_y);

  propogateManhattanDistances();

  return true;
}

bool GoalDistCritic::getLastPoseOnCostmap(
    const nav_2d_msgs::Path2D& global_plan, unsigned int& x, unsigned int& y) {
  nav_2d_msgs::Path2D adjusted_global_plan =
      dwb_core::adjustPlanResolution(global_plan, costmap_->getResolution());
  bool started_path = false;

  // skip global path points until we reach the border of the local map
  for (unsigned int i = 0; i < adjusted_global_plan.poses.size(); ++i) {
    double g_x = adjusted_global_plan.poses[i].x;
    double g_y = adjusted_global_plan.poses[i].y;
    unsigned int map_x, map_y;
    if (costmap_->worldToMap(g_x, g_y, map_x, map_y) &&
        costmap_->getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
      // Still on the costmap. Continue.
      x = map_x;
      y = map_y;
      started_path = true;
    } else if (started_path) {
      // Off the costmap after being on the costmap. Return the last saved
      // indices.
      return true;
    }
    // else, we have not yet found a point on the costmap, so we just continue
  }

  if (started_path) {
    return true;
  } else {
    spdlog::error("None of the points of the global plan were in the local "
                  "costmap.");
    return false;
  }
}

}  // namespace dwb_critics
