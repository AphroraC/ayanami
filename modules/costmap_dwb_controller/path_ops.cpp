// path_ops.cpp — 迁移自 nav_2d_utils/src/path_ops.cpp (纯算法)。
#include "path_ops.hpp"

#include <cmath>

using std::sqrt;

namespace dwb_core {

nav_2d_msgs::Path2D adjustPlanResolution(
    const nav_2d_msgs::Path2D& global_plan_in, double resolution) {
  nav_2d_msgs::Path2D global_plan_out;
  if (global_plan_in.poses.size() == 0) {
    return global_plan_out;
  }

  geometry_msgs::Pose2D last = global_plan_in.poses[0];
  global_plan_out.poses.push_back(last);

  // we can take "holes" in the plan smaller than 2 grid cells (squared = 4)
  double min_sq_resolution = resolution * resolution * 4.0;

  for (unsigned int i = 1; i < global_plan_in.poses.size(); ++i) {
    geometry_msgs::Pose2D loop = global_plan_in.poses[i];
    double sq_dist = (loop.x - last.x) * (loop.x - last.x) +
                     (loop.y - last.y) * (loop.y - last.y);
    if (sq_dist > min_sq_resolution) {
      // add points in-between
      double diff = sqrt(sq_dist) - sqrt(min_sq_resolution);
      int steps = static_cast<int>(diff / resolution) - 1;
      double steps_double = static_cast<double>(steps);

      double delta_x = (loop.x - last.x) / steps_double;
      double delta_y = (loop.y - last.y) / steps_double;
      double delta_t = (loop.theta - last.theta) / steps_double;

      for (int j = 1; j < steps; ++j) {
        geometry_msgs::Pose2D pose;
        pose.x = last.x + j * delta_x;
        pose.y = last.y + j * delta_y;
        pose.theta = last.theta + j * delta_t;
        global_plan_out.poses.push_back(pose);
      }
    }
    global_plan_out.poses.push_back(global_plan_in.poses[i]);
    last.x = loop.x;
    last.y = loop.y;
  }
  return global_plan_out;
}

}  // namespace dwb_core
