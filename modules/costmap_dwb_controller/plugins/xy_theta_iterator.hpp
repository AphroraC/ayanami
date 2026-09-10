// xy_theta_iterator.hpp -- ported from dwb_plugins/xy_theta_iterator.hpp.
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
      : kinematics_handler(nullptr), x_it(nullptr), y_it(nullptr),
        th_it(nullptr) {}
  void initialize(const ayanami::Config& cfg, KinematicsHandler::Ptr kinematics,
                  const std::string& module_name) override;
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
  int vx_samples, vy_samples, vtheta_samples;
  KinematicsHandler::Ptr kinematics_handler;

  std::shared_ptr<OneDVelocityIterator> x_it, y_it, th_it;
};
}  // namespace dwb_plugins
