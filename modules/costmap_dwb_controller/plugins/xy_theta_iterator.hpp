// xy_theta_iterator.hpp — 迁移自 dwb_plugins/xy_theta_iterator.hpp。
#pragma once

#include <memory>
#include <string>

#include "ayanami/utility/config.hpp"

#include "velocity_iterator.hpp"
#include "one_d_velocity_iterator.hpp"

namespace dwb_plugins {
class XYThetaIterator : public VelocityIterator {
public:
  XYThetaIterator()
      : kinematics_handler_(nullptr), x_it_(nullptr), y_it_(nullptr),
        th_it_(nullptr) {}
  void initialize(const ayanami::Config& cfg, KinematicsHandler::Ptr kinematics,
                  const std::string& plugin_name) override;
  void startNewIteration(const nav_2d_msgs::Twist2D& current_velocity,
                         double dt) override;
  bool hasMoreTwists() override;
  nav_2d_msgs::Twist2D nextTwist() override;

protected:
  /**
   * @brief Check to see whether the combined x/y/theta velocities are valid
   * @return True if the magnitude hypot(x,y) and theta are within the robot's
   * absolute limits
   */
  bool isValidSpeed(double x, double y, double theta);
  virtual bool isValidVelocity();
  void iterateToValidVelocity();
  int vx_samples_, vy_samples_, vtheta_samples_;
  KinematicsHandler::Ptr kinematics_handler_;

  std::shared_ptr<OneDVelocityIterator> x_it_, y_it_, th_it_;
};
}  // namespace dwb_plugins
