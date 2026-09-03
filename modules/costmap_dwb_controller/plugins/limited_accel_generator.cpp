// limited_accel_generator.cpp — 迁移自 dwb_plugins/src/
// limited_accel_generator.cpp (sim_period 回退 controller_frequency 逻辑保留,
// controller_frequency 从 config 读)。
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

  // 原版: sim_period 参数声明失败 (未设置) → 回退 controller_frequency。
  // Config json 无"参数未声明"概念, 以 hasParam 判缺省。
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
