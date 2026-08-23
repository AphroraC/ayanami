// twirling.cpp — 迁移自 dwb_critics/src/twirling.cpp。
#include "twirling.hpp"

namespace dwb_critics {
void TwirlingCritic::onInit() {
  // Scale is set to 0 by default, so if it was not set otherwise, set to 0
  scale_ = config().param<double>(dwb_plugin_name_, name_ + ".scale", 1.0);
}

double TwirlingCritic::scoreTrajectory(
    const dwb_msgs::Trajectory2D& traj) {
  return fabs(traj.velocity.theta);  // add cost for making the robot spin
}
}  // namespace dwb_critics
