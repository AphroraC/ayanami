// standard_traj_generator.hpp -- ported from dwb_plugins/standard_traj_generator.hpp.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "ayanami/utility/config.hpp"

#include "../dwb_types.hpp"
#include "../trajectory_generator.hpp"
#include "velocity_iterator.hpp"
#include "kinematic_parameters.hpp"

namespace dwb_plugins {

/**
 * @class StandardTrajectoryGenerator
 * @brief Standard DWA-like trajectory generator.
 */
class StandardTrajectoryGenerator : public dwb_core::TrajectoryGenerator {
public:
  // Standard TrajectoryGenerator interface
  void initialize(const ayanami::Config& cfg,
                  const std::string& module_name) override;
  void startNewIteration(
      const nav_2d_msgs::Twist2D& current_velocity) override;
  bool hasMoreTwists() override;
  nav_2d_msgs::Twist2D nextTwist() override;

  dwb_msgs::Trajectory2D generateTrajectory(
      const geometry_msgs::Pose2D& start_pose,
      const nav_2d_msgs::Twist2D& start_vel,
      const nav_2d_msgs::Twist2D& cmd_vel) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   */
  void setSpeedLimit(const double& speed_limit,
                     const bool& percentage) override {
    if (kinematics_handler) {
      kinematics_handler->setSpeedLimit(speed_limit, percentage);
    }
  }

protected:
  /**
   * @brief Initialize the VelocityIterator pointer. Put in its own function for
   * easy overriding
   */
  virtual void initializeIterator(const ayanami::Config& cfg);

  /**
   * @brief Calculate the velocity after a set period of time, given the desired
   * velocity and acceleration limits
   */
  virtual nav_2d_msgs::Twist2D computeNewVelocity(
      const nav_2d_msgs::Twist2D& cmd_vel, const nav_2d_msgs::Twist2D& start_vel,
      const double dt);

  /**
   * @brief Use the robot's kinematic model to predict new positions for the
   * robot
   */
  virtual geometry_msgs::Pose2D computeNewPosition(
      const geometry_msgs::Pose2D start_pose, const nav_2d_msgs::Twist2D& vel,
      const double dt);

  /**
   * @brief Compute an array of time deltas between the points in the generated
   * trajectory.
   */
  virtual std::vector<double> getTimeSteps(const nav_2d_msgs::Twist2D& cmd_vel);

  KinematicsHandler::Ptr kinematics_handler;
  std::shared_ptr<VelocityIterator> velocity_iterator;

  double sim_time;

  // Sampling Parameters
  bool discretize_by_time;

  /// @brief If discretizing by time, the amount of time between each point in
  /// the traj
  double time_granularity;

  /// @brief If discretizing by distance, the maximum distance between each
  /// point in the traj
  double linear_granularity;

  /// @brief If discretizing by distance, the maximum angular distance between
  /// each point in the traj
  double angular_granularity;

  /// @brief Whether to include the last point of the trajectory
  bool include_last_point;

  /// @brief Whether to limit the velocity command in the trajectory
  bool limit_vel_cmd_in_traj;

  std::string module_name;
};

}  // namespace dwb_plugins
