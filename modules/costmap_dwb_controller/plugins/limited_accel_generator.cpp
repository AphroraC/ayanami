// limited_accel_generator.cpp -- ported from dwb_plugins/src/
// limited_accel_generator.cpp (the sim_period fallback-to-controller_frequency logic is kept,
// controller_frequency read from config).
#include "limited_accel_generator.hpp"

#include <memory>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace dwb_plugins {

void LimitedAccelGenerator::initialize(const ayanami::Config& cfg,
                                       const std::string& plugin_name) {
  this->plugin_name = plugin_name;
  StandardTrajectoryGenerator::initialize(cfg, plugin_name);

  // Original: if the sim_period parameter declaration fails (unset) -> fall back to controller_frequency.
  // Config json has no "undeclared parameter" notion, so absence is detected with hasParam.
  if (cfg.hasParam(plugin_name, "sim_period")) {
    acceleration_time = cfg.param<double>(plugin_name, "sim_period", 0.0);
  } else {
    spdlog::warn("'sim_period' parameter is not set for {}", plugin_name);
    double controller_frequency =
        cfg.param<double>(plugin_name, "controller_frequency", 20.0);
    if (controller_frequency > 0) {
      acceleration_time = 1.0 / controller_frequency;
    } else {
      spdlog::warn("A controller_frequency less than or equal to 0 has been "
                   "set. Ignoring the parameter, assuming a rate of 20Hz");
      acceleration_time = 0.05;
    }
  }
}

void LimitedAccelGenerator::startNewIteration(
    const nav_2d_msgs::Twist2D& current_velocity) {
  // Limit our search space to just those within the limited acceleration_time
  velocity_iterator->startNewIteration(current_velocity, acceleration_time);
}

nav_2d_msgs::Twist2D LimitedAccelGenerator::computeNewVelocity(
    const nav_2d_msgs::Twist2D& cmd_vel,
    const nav_2d_msgs::Twist2D& /*start_vel*/, const double /*dt*/) {
  return cmd_vel;
}

}  // namespace dwb_plugins
