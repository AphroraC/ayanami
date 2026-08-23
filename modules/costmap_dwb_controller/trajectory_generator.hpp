// trajectory_generator.hpp — 迁移自 dwb_core/trajectory_generator.hpp
// (LifecycleNode 参数读取换 ayanami::Config)。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "ayanami/utility/config.hpp"

#include "dwb_types.hpp"

namespace dwb_core {

/**
 * @class TrajectoryGenerator
 * @brief Interface for iterating through possible velocities and creating
 * trajectories
 */
class TrajectoryGenerator {
public:
  typedef std::shared_ptr<dwb_core::TrajectoryGenerator> Ptr;

  virtual ~TrajectoryGenerator() {}

  /**
   * @brief Initialize parameters as needed
   * @param cfg Config to read parameters from (json 段: plugin_name)
   * @param plugin_name The parameter namespace of this plugin
   */
  virtual void initialize(const ayanami::Config& cfg,
                          const std::string& plugin_name) = 0;
  virtual void reset() {}
  /**
   * @brief Start a new iteration based on the current velocity
   * @param current_velocity
   */
  virtual void startNewIteration(
      const nav_2d_msgs::Twist2D& current_velocity) = 0;

  /**
   * @brief Test to see whether there are more twists to test
   */
  virtual bool hasMoreTwists() = 0;

  /**
   * @brief Return the next twist and advance the iteration
   */
  virtual nav_2d_msgs::Twist2D nextTwist() = 0;

  /**
   * @brief Get all the twists for an iteration.
   */
  virtual std::vector<nav_2d_msgs::Twist2D> getTwists(
      const nav_2d_msgs::Twist2D& current_velocity) {
    std::vector<nav_2d_msgs::Twist2D> twists;
    startNewIteration(current_velocity);
    while (hasMoreTwists()) {
      twists.push_back(nextTwist());
    }
    return twists;
  }

  /**
   * @brief Given a cmd_vel in the robot's frame and initial conditions,
   * generate a Trajectory2D
   */
  virtual dwb_msgs::Trajectory2D generateTrajectory(
      const geometry_msgs::Pose2D& start_pose,
      const nav_2d_msgs::Twist2D& start_vel,
      const nav_2d_msgs::Twist2D& cmd_vel) = 0;

  /**
   * @brief Limits the maximum linear speed of the robot.
   */
  virtual void setSpeedLimit(const double& speed_limit,
                             const bool& percentage) = 0;
};

}  // namespace dwb_core
