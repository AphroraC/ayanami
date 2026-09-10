// path_align.cpp -- ported from dwb_critics/src/path_align.cpp.
#include "path_align.hpp"

#include <string>
#include <vector>

#include "alignment_util.hpp"

namespace dwb_critics {

void PathAlignCritic::onInit() {
  PathDistCritic::onInit();
  stop_on_failure = false;

  forward_point_distance =
      config().param<double>(dwb_module_name,
                             name + ".forward_point_distance", 0.325);
}

bool PathAlignCritic::prepare(const geometry_msgs::Pose2D& pose,
                              const nav_2d_msgs::Twist2D& vel,
                              const geometry_msgs::Pose2D& goal,
                              const nav_2d_msgs::Path2D& global_plan) {
  double dx = pose.x - goal.x;
  double dy = pose.y - goal.y;
  double sq_dist = dx * dx + dy * dy;
  if (sq_dist > forward_point_distance * forward_point_distance) {
    zero_scale = false;
  } else {
    // once we are close to goal, trying to keep the nose close to anything
    // destabilizes behavior.
    zero_scale = true;
    return true;
  }

  return PathDistCritic::prepare(pose, vel, goal, global_plan);
}

double PathAlignCritic::getScale() const {
  if (zero_scale) {
    return 0.0;
  } else {
    return costmap->getResolution() * 0.5 * scale;
  }
}

double PathAlignCritic::scorePose(const geometry_msgs::Pose2D& pose) {
  return PathDistCritic::scorePose(getForwardPose(pose, forward_point_distance));
}

}  // namespace dwb_critics
