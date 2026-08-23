// prefer_forward.cpp — 迁移自 dwb_critics/src/prefer_forward.cpp。
#include "prefer_forward.hpp"

#include <math.h>

namespace dwb_critics {

void PreferForwardCritic::onInit() {
  penalty_ = config().param<double>(dwb_plugin_name_, name_ + ".penalty", 1.0);
  strafe_x_ = config().param<double>(dwb_plugin_name_, name_ + ".strafe_x", 0.1);
  strafe_theta_ =
      config().param<double>(dwb_plugin_name_, name_ + ".strafe_theta", 0.2);
  theta_scale_ =
      config().param<double>(dwb_plugin_name_, name_ + ".theta_scale", 10.0);
}

double PreferForwardCritic::scoreTrajectory(
    const dwb_msgs::Trajectory2D& traj) {
  // backward motions bad on a robot without backward sensors
  if (traj.velocity.x < 0.0) {
    return penalty_;
  }
  // strafing motions also bad on such a robot
  if (traj.velocity.x < strafe_x_ && fabs(traj.velocity.theta) < strafe_theta_) {
    return penalty_;
  }

  // the more we rotate, the less we progress forward
  return fabs(traj.velocity.theta) * theta_scale_;
}

}  // namespace dwb_critics
