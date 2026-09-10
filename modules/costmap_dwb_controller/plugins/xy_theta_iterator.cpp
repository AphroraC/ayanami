// xy_theta_iterator.cpp -- ported from dwb_plugins/src/xy_theta_iterator.cpp.
#include "xy_theta_iterator.hpp"

#include <cmath>
#include <memory>
#include <string>

#define EPSILON 1E-5

namespace dwb_plugins {
void XYThetaIterator::initialize(const ayanami::Config& cfg,
                                 KinematicsHandler::Ptr kinematics,
                                 const std::string& module_name) {
  kinematics_handler = kinematics;

  vx_samples = cfg.param<int>(module_name, "vx_samples", 20);
  vy_samples = cfg.param<int>(module_name, "vy_samples", 5);
  vtheta_samples = cfg.param<int>(module_name, "vtheta_samples", 20);
}

void XYThetaIterator::startNewIteration(
    const nav_2d_msgs::Twist2D& current_velocity, double dt) {
  KinematicParameters kinematics = kinematics_handler->getKinematics();
  x_it = std::make_shared<OneDVelocityIterator>(
      current_velocity.x, kinematics.getMinX(), kinematics.getMaxX(),
      kinematics.getAccX(), kinematics.getDecelX(), dt, vx_samples);
  y_it = std::make_shared<OneDVelocityIterator>(
      current_velocity.y, kinematics.getMinY(), kinematics.getMaxY(),
      kinematics.getAccY(), kinematics.getDecelY(), dt, vy_samples);
  th_it = std::make_shared<OneDVelocityIterator>(
      current_velocity.theta, kinematics.getMinTheta(),
      kinematics.getMaxTheta(), kinematics.getAccTheta(),
      kinematics.getDecelTheta(), dt, vtheta_samples);
  if (!isValidVelocity()) {
    iterateToValidVelocity();
  }
}

bool XYThetaIterator::isValidSpeed(double x, double y, double theta) {
  KinematicParameters kinematics = kinematics_handler->getKinematics();
  double vmag_sq = x * x + y * y;
  if (kinematics.getMaxSpeedXY() >= 0.0 &&
      vmag_sq > kinematics.getMaxSpeedXY_SQ() + EPSILON) {
    return false;
  }
  if (kinematics.getMinSpeedXY() >= 0.0 &&
      vmag_sq + EPSILON < kinematics.getMinSpeedXY_SQ() &&
      kinematics.getMinSpeedTheta() >= 0.0 &&
      fabs(theta) + EPSILON < kinematics.getMinSpeedTheta()) {
    return false;
  }
  if (vmag_sq == 0.0 && th_it->getVelocity() == 0.0) {
    return false;
  }
  return true;
}

bool XYThetaIterator::isValidVelocity() {
  return isValidSpeed(x_it->getVelocity(), y_it->getVelocity(),
                      th_it->getVelocity());
}

bool XYThetaIterator::hasMoreTwists() {
  return x_it && !x_it->isFinished();
}

nav_2d_msgs::Twist2D XYThetaIterator::nextTwist() {
  nav_2d_msgs::Twist2D velocity;
  velocity.x = x_it->getVelocity();
  velocity.y = y_it->getVelocity();
  velocity.theta = th_it->getVelocity();

  iterateToValidVelocity();

  return velocity;
}

void XYThetaIterator::iterateToValidVelocity() {
  bool valid = false;
  while (!valid && hasMoreTwists()) {
    ++(*th_it);
    if (th_it->isFinished()) {
      th_it->reset();
      ++(*y_it);
      if (y_it->isFinished()) {
        y_it->reset();
        ++(*x_it);
      }
    }
    valid = isValidVelocity();
  }
}

}  // namespace dwb_plugins
