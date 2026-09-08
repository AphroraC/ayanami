// velocity_iterator.hpp -- ported from dwb_plugins/velocity_iterator.hpp
// (LifecycleNode becomes ayanami::Config).
#pragma once

#include <memory>
#include <string>

#include "ayanami/utility/config.hpp"

#include "dwb_types.hpp"
#include "kinematic_parameters.hpp"

namespace dwb_plugins {
class VelocityIterator {
public:
  virtual ~VelocityIterator() {}
  virtual void initialize(const ayanami::Config& cfg,
                          KinematicsHandler::Ptr kinematics,
                          const std::string& plugin_name) = 0;
  virtual void startNewIteration(const nav_2d_msgs::Twist2D& current_velocity,
                                 double dt) = 0;
  virtual bool hasMoreTwists() = 0;
  virtual nav_2d_msgs::Twist2D nextTwist() = 0;
};
}  // namespace dwb_plugins
