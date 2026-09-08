// prefer_forward.cpp -- ported from dwb_critics/src/prefer_forward.cpp.
#include "prefer_forward.hpp"

#include <math.h>

namespace dwb_critics {

void PreferForwardCritic::onInit() {
  penalty = config().param<double>(dwb_plugin_name, name + ".penalty", 1.0);
  strafe_x = config().param<double>(dwb_plugin_name, name + ".strafe_x", 0.1);
  strafe_theta =
      config().param<double>(dwb_plugin_name, name + ".strafe_theta", 0.2);
  theta_scale =
      config().param<double>(dwb_plugin_name, name + ".theta_scale", 10.0);
}

double PreferForwardCritic::scoreTrajectory(
    const dwb_msgs::Trajectory2D& traj) {
  // backward motions bad on a robot without backward sensors
  if (traj.velocity.x < 0.0) {
    return penalty;
  }
  // strafing motions also bad on such a robot
  if (traj.velocity.x < strafe_x && fabs(traj.velocity.theta) < strafe_theta) {
    return penalty;
  }

  // the more we rotate, the less we progress forward
  return fabs(traj.velocity.theta) * theta_scale;
}

}  // namespace dwb_critics
