// alignment_util.cpp -- ported from dwb_critics/src/alignment_util.cpp.
#include "alignment_util.hpp"

#include <cmath>

using std::cos;
using std::sin;

namespace dwb_critics {
geometry_msgs::Pose2D getForwardPose(const geometry_msgs::Pose2D& pose,
                                     double distance) {
  geometry_msgs::Pose2D forward_pose;
  forward_pose.x = pose.x + distance * cos(pose.theta);
  forward_pose.y = pose.y + distance * sin(pose.theta);
  forward_pose.theta = pose.theta;
  return forward_pose;
}
}  // namespace dwb_critics
