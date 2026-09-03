// kinematic_parameters.cpp — 迁移自 dwb_plugins/src/kinematic_parameters.cpp
// (rclcpp 参数声明换 Config::param; 移除动态参数回调, 参数一次性读取)。
#include "kinematic_parameters.hpp"

#include <string>

namespace dwb_plugins {

KinematicsHandler::KinematicsHandler() {}

KinematicsHandler::~KinematicsHandler() {}

void KinematicsHandler::initialize(const ayanami::Config& cfg,
                                   const std::string& plugin_name) {
  this->plugin_name = plugin_name;

  kinematics.min_vel_x = cfg.param<double>(plugin_name, "min_vel_x", 0.0);
  kinematics.min_vel_y = cfg.param<double>(plugin_name, "min_vel_y", 0.0);
  kinematics.max_vel_x = cfg.param<double>(plugin_name, "max_vel_x", 0.0);
  kinematics.max_vel_y = cfg.param<double>(plugin_name, "max_vel_y", 0.0);
  kinematics.max_vel_theta =
      cfg.param<double>(plugin_name, "max_vel_theta", 0.0);
  kinematics.min_speed_xy =
      cfg.param<double>(plugin_name, "min_speed_xy", 0.0);
  kinematics.max_speed_xy =
      cfg.param<double>(plugin_name, "max_speed_xy", 0.0);
  kinematics.min_speed_theta =
      cfg.param<double>(plugin_name, "min_speed_theta", 0.0);
  kinematics.acc_lim_x = cfg.param<double>(plugin_name, "acc_lim_x", 0.0);
  kinematics.acc_lim_y = cfg.param<double>(plugin_name, "acc_lim_y", 0.0);
  kinematics.acc_lim_theta =
      cfg.param<double>(plugin_name, "acc_lim_theta", 0.0);
  kinematics.decel_lim_x =
      cfg.param<double>(plugin_name, "decel_lim_x", 0.0);
  kinematics.decel_lim_y =
      cfg.param<double>(plugin_name, "decel_lim_y", 0.0);
  kinematics.decel_lim_theta =
      cfg.param<double>(plugin_name, "decel_lim_theta", 0.0);

  kinematics.base_max_vel_x = kinematics.max_vel_x;
  kinematics.base_max_vel_y = kinematics.max_vel_y;
  kinematics.base_max_speed_xy = kinematics.max_speed_xy;
  kinematics.base_max_vel_theta = kinematics.max_vel_theta;

  kinematics.min_speed_xy_sq =
      kinematics.min_speed_xy * kinematics.min_speed_xy;
  kinematics.max_speed_xy_sq =
      kinematics.max_speed_xy * kinematics.max_speed_xy;
}

void KinematicsHandler::setSpeedLimit(const double& speed_limit,
                                      const bool& percentage) {
  KinematicParameters kinematics(kinematics);

  if (speed_limit == NO_SPEED_LIMIT) {
    // Restore default value
    kinematics.max_speed_xy = kinematics.base_max_speed_xy;
    kinematics.max_vel_x = kinematics.base_max_vel_x;
    kinematics.max_vel_y = kinematics.base_max_vel_y;
    kinematics.max_vel_theta = kinematics.base_max_vel_theta;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      kinematics.max_speed_xy = kinematics.base_max_speed_xy * speed_limit / 100.0;
      kinematics.max_vel_x = kinematics.base_max_vel_x * speed_limit / 100.0;
      kinematics.max_vel_y = kinematics.base_max_vel_y * speed_limit / 100.0;
      kinematics.max_vel_theta = kinematics.base_max_vel_theta * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      if (speed_limit < kinematics.base_max_speed_xy) {
        kinematics.max_speed_xy = speed_limit;
        // Handling components and angular velocity changes:
        // Max velocities are being changed in the same proportion
        // as absolute linear speed changed in order to preserve
        // robot moving trajectories to be the same after speed change.
        const double ratio = speed_limit / kinematics.base_max_speed_xy;
        kinematics.max_vel_x = kinematics.base_max_vel_x * ratio;
        kinematics.max_vel_y = kinematics.base_max_vel_y * ratio;
        kinematics.max_vel_theta = kinematics.base_max_vel_theta * ratio;
      }
    }
  }

  // Do not forget to update max_speed_xy_sq as well
  kinematics.max_speed_xy_sq = kinematics.max_speed_xy * kinematics.max_speed_xy;

  this->kinematics = kinematics;
}

}  // namespace dwb_plugins
