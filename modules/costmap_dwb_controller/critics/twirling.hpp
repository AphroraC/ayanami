// twirling.hpp -- ported from dwb_critics/twirling.hpp.
#pragma once

#include <string>
#include <vector>

#include "../trajectory_critic.hpp"

namespace dwb_critics {

/**
 * @class TwirlingCritic
 * @brief Scores trajectories based on the rotational velocity.
 *
 * This critic is meant to prevent the robot from twirling on the spot, since
 * it can take a while to stop spinning once it starts.
 */
class TwirlingCritic : public dwb_core::TrajectoryCritic {
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;
};

}  // namespace dwb_critics
