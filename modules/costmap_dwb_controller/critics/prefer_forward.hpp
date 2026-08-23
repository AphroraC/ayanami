// prefer_forward.hpp — 迁移自 dwb_critics/prefer_forward.hpp。
#pragma once

#include <string>
#include <vector>

#include "../trajectory_critic.hpp"

namespace dwb_critics {

/**
 * @class PreferForwardCritic
 * @brief Penalizes trajectories that move backward or strafe.
 */
class PreferForwardCritic : public dwb_core::TrajectoryCritic {
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;

protected:
  double penalty_, strafe_x_, strafe_theta_, theta_scale_;
};

}  // namespace dwb_critics
