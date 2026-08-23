// xy_theta_iterator.cpp — 迁移自 dwb_plugins/src/xy_theta_iterator.cpp。
#include "xy_theta_iterator.hpp"

#include <cmath>
#include <memory>
#include <string>

#define EPSILON 1E-5

namespace dwb_plugins {
void XYThetaIterator::initialize(const ayanami::Config& cfg,
                                 KinematicsHandler::Ptr kinematics,
                                 const std::string& plugin_name) {
  kinematics_handler_ = kinematics;

  vx_samples_ = cfg.param<int>(plugin_name, "vx_samples", 20);
  vy_samples_ = cfg.param<int>(plugin_name, "vy_samples", 5);
  vtheta_samples_ = cfg.param<int>(plugin_name, "vtheta_samples", 20);
}

void XYThetaIterator::startNewIteration(
    const nav_2d_msgs::Twist2D& current_velocity, double dt) {
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
  x_it_ = std::make_shared<OneDVelocityIterator>(
      current_velocity.x, kinematics.getMinX(), kinematics.getMaxX(),
      kinematics.getAccX(), kinematics.getDecelX(), dt, vx_samples_);
  y_it_ = std::make_shared<OneDVelocityIterator>(
      current_velocity.y, kinematics.getMinY(), kinematics.getMaxY(),
      kinematics.getAccY(), kinematics.getDecelY(), dt, vy_samples_);
  th_it_ = std::make_shared<OneDVelocityIterator>(
      current_velocity.theta, kinematics.getMinTheta(),
      kinematics.getMaxTheta(), kinematics.getAccTheta(),
      kinematics.getDecelTheta(), dt, vtheta_samples_);
  if (!isValidVelocity()) {
    iterateToValidVelocity();
  }
}

bool XYThetaIterator::isValidSpeed(double x, double y, double theta) {
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
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
  if (vmag_sq == 0.0 && th_it_->getVelocity() == 0.0) {
    return false;
  }
  return true;
}

bool XYThetaIterator::isValidVelocity() {
  return isValidSpeed(x_it_->getVelocity(), y_it_->getVelocity(),
                      th_it_->getVelocity());
}

bool XYThetaIterator::hasMoreTwists() {
  return x_it_ && !x_it_->isFinished();
}

nav_2d_msgs::Twist2D XYThetaIterator::nextTwist() {
  nav_2d_msgs::Twist2D velocity;
  velocity.x = x_it_->getVelocity();
  velocity.y = y_it_->getVelocity();
  velocity.theta = th_it_->getVelocity();

  iterateToValidVelocity();

  return velocity;
}

void XYThetaIterator::iterateToValidVelocity() {
  bool valid = false;
  while (!valid && hasMoreTwists()) {
    ++(*th_it_);
    if (th_it_->isFinished()) {
      th_it_->reset();
      ++(*y_it_);
      if (y_it_->isFinished()) {
        y_it_->reset();
        ++(*x_it_);
      }
    }
    valid = isValidVelocity();
  }
}

}  // namespace dwb_plugins
