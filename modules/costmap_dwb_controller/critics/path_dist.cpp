// path_dist.cpp -- ported from dwb_critics/src/path_dist.cpp.
#include "path_dist.hpp"

#include <vector>

#include <spdlog/spdlog.h>

#include <costmap_2d/cost_values.hpp>

#include "../path_ops.hpp"

namespace dwb_critics {
bool PathDistCritic::prepare(const geometry_msgs::Pose2D&,
                             const nav_2d_msgs::Twist2D&,
                             const geometry_msgs::Pose2D&,
                             const nav_2d_msgs::Path2D& global_plan) {
  reset();
  bool started_path = false;

  nav_2d_msgs::Path2D adjusted_global_plan =
      dwb_core::adjustPlanResolution(global_plan, costmap->getResolution());

  if (adjusted_global_plan.poses.size() != global_plan.poses.size()) {
    spdlog::debug("Adjusted global plan resolution, added {} points",
                  adjusted_global_plan.poses.size() -
                      global_plan.poses.size());
  }

  unsigned int i;
  // put global path points into local map until we reach the border of the
  // local map
  for (i = 0; i < adjusted_global_plan.poses.size(); ++i) {
    double g_x = adjusted_global_plan.poses[i].x;
    double g_y = adjusted_global_plan.poses[i].y;
    unsigned int map_x, map_y;
    if (costmap->worldToMap(g_x, g_y, map_x, map_y) &&
        costmap->getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
      int index = costmap->getIndex(map_x, map_y);
      cell_values[index] = 0.0;
      queue->enqueueCell(map_x, map_y);
      started_path = true;
    } else if (started_path) {
      break;
    }
  }
  if (!started_path) {
    spdlog::error(
        "None of the {} first of {} ({}) points of the global plan were in "
        "the local costmap and free",
        i, adjusted_global_plan.poses.size(), global_plan.poses.size());
    return false;
  }

  propogateManhattanDistances();

  return true;
}

}  // namespace dwb_critics
