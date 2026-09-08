// limited_accel_generator.hpp -- ported from dwb_plugins/limited_accel_generator.hpp.
#pragma once

#include <memory>
#include <string>

#include "ayanami/utility/config.hpp"

#include "standard_traj_generator.hpp"

namespace dwb_plugins {
/**
 * @class LimitedAccelGenerator
 * @brief Limits the acceleration in the generated trajectories to a fraction
 * of the simulated time.
 */
class LimitedAccelGenerator : public StandardTrajectoryGenerator {
public:
  void initialize(const ayanami::Config& cfg,
                  const std::string& plugin_name) override;
  void startNewIteration(
      const nav_2d_msgs::Twist2D& current_velocity) override;

protected:
  /**
   * @brief Calculate the velocity after a set period of time, given the desired
   * velocity and acceleration limits
   *
   * Unlike the StandardTrajectoryGenerator, the velocity remains constant in
   * the LimitedAccelGenerator
   */
  nav_2d_msgs::Twist2D computeNewVelocity(
      const nav_2d_msgs::Twist2D& cmd_vel,
      const nav_2d_msgs::Twist2D& start_vel, const double dt) override;
  double acceleration_time;
  std::string plugin_name;
};
}  // namespace dwb_plugins
