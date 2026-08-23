// kinematic_parameters.cpp — 迁移自 dwb_plugins/src/kinematic_parameters.cpp
// (rclcpp 参数声明换 Config::param; 移除动态参数回调, 参数一次性读取)。
#include "kinematic_parameters.hpp"

#include <string>

namespace dwb_plugins {

KinematicsHandler::KinematicsHandler() {}

KinematicsHandler::~KinematicsHandler() {}

void KinematicsHandler::initialize(const ayanami::Config& cfg,
                                   const std::string& plugin_name) {
  plugin_name_ = plugin_name;

  kinematics_.min_vel_x_ = cfg.param<double>(plugin_name, "min_vel_x", 0.0);
  kinematics_.min_vel_y_ = cfg.param<double>(plugin_name, "min_vel_y", 0.0);
  kinematics_.max_vel_x_ = cfg.param<double>(plugin_name, "max_vel_x", 0.0);
  kinematics_.max_vel_y_ = cfg.param<double>(plugin_name, "max_vel_y", 0.0);
  kinematics_.max_vel_theta_ =
      cfg.param<double>(plugin_name, "max_vel_theta", 0.0);
  kinematics_.min_speed_xy_ =
      cfg.param<double>(plugin_name, "min_speed_xy", 0.0);
  kinematics_.max_speed_xy_ =
      cfg.param<double>(plugin_name, "max_speed_xy", 0.0);
  kinematics_.min_speed_theta_ =
      cfg.param<double>(plugin_name, "min_speed_theta", 0.0);
  kinematics_.acc_lim_x_ = cfg.param<double>(plugin_name, "acc_lim_x", 0.0);
  kinematics_.acc_lim_y_ = cfg.param<double>(plugin_name, "acc_lim_y", 0.0);
  kinematics_.acc_lim_theta_ =
      cfg.param<double>(plugin_name, "acc_lim_theta", 0.0);
  kinematics_.decel_lim_x_ =
      cfg.param<double>(plugin_name, "decel_lim_x", 0.0);
  kinematics_.decel_lim_y_ =
      cfg.param<double>(plugin_name, "decel_lim_y", 0.0);
  kinematics_.decel_lim_theta_ =
      cfg.param<double>(plugin_name, "decel_lim_theta", 0.0);

  kinematics_.base_max_vel_x_ = kinematics_.max_vel_x_;
  kinematics_.base_max_vel_y_ = kinematics_.max_vel_y_;
  kinematics_.base_max_speed_xy_ = kinematics_.max_speed_xy_;
  kinematics_.base_max_vel_theta_ = kinematics_.max_vel_theta_;

  kinematics_.min_speed_xy_sq_ =
      kinematics_.min_speed_xy_ * kinematics_.min_speed_xy_;
  kinematics_.max_speed_xy_sq_ =
      kinematics_.max_speed_xy_ * kinematics_.max_speed_xy_;
}

void KinematicsHandler::setSpeedLimit(const double& speed_limit,
                                      const bool& percentage) {
  KinematicParameters kinematics(kinematics_);

  if (speed_limit == NO_SPEED_LIMIT) {
    // Restore default value
    kinematics.max_speed_xy_ = kinematics.base_max_speed_xy_;
    kinematics.max_vel_x_ = kinematics.base_max_vel_x_;
    kinematics.max_vel_y_ = kinematics.base_max_vel_y_;
    kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      kinematics.max_speed_xy_ = kinematics.base_max_speed_xy_ * speed_limit / 100.0;
      kinematics.max_vel_x_ = kinematics.base_max_vel_x_ * speed_limit / 100.0;
      kinematics.max_vel_y_ = kinematics.base_max_vel_y_ * speed_limit / 100.0;
      kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      if (speed_limit < kinematics.base_max_speed_xy_) {
        kinematics.max_speed_xy_ = speed_limit;
        // Handling components and angular velocity changes:
        // Max velocities are being changed in the same proportion
        // as absolute linear speed changed in order to preserve
        // robot moving trajectories to be the same after speed change.
        const double ratio = speed_limit / kinematics.base_max_speed_xy_;
        kinematics.max_vel_x_ = kinematics.base_max_vel_x_ * ratio;
        kinematics.max_vel_y_ = kinematics.base_max_vel_y_ * ratio;
        kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_ * ratio;
      }
    }
  }

  // Do not forget to update max_speed_xy_sq_ as well
  kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;

  kinematics_ = kinematics;
}

}  // namespace dwb_plugins
