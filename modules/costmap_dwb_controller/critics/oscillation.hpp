// oscillation.hpp — 迁移自 dwb_critics/oscillation.hpp (rclcpp::Duration/
// Clock 换 double 秒 + std::chrono steady_clock)。
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "../trajectory_critic.hpp"

namespace dwb_critics {

/**
 * @class OscillationCritic
 * @brief Checks to see whether the sign of the commanded velocity flips
 * frequently
 */
class OscillationCritic : public dwb_core::TrajectoryCritic {
public:
  OscillationCritic() : oscillation_reset_time_(-1.0) {}
  void onInit() override;
  bool prepare(const geometry_msgs::Pose2D& pose,
               const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal,
               const nav_2d_msgs::Path2D& global_plan) override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;
  void reset() override;
  void debrief(const nav_2d_msgs::Twist2D& cmd_vel) override;

private:
  /**
   * @class CommandTrend
   * @brief Helper class for performing the same logic on the x,y and theta
   * dimensions
   */
  class CommandTrend {
public:
    CommandTrend();
    void reset();

    /**
     * @brief update internal flags based on the commanded velocity
     */
    bool update(double velocity);

    /**
     * @brief Check to see whether the proposed velocity would be considered
     * oscillating
     */
    bool isOscillating(double velocity);

    /**
     * @brief Check whether we are currently tracking a flipped sign
     */
    bool hasSignFlipped();

private:
    // Simple Enum for Tracking
    enum class Sign { ZERO, POSITIVE, NEGATIVE };

    Sign sign_;
    bool positive_only_, negative_only_;
  };

  /**
   * @brief Given a command that has been selected, track each component's sign
   * for oscillations
   */
  bool setOscillationFlags(const nav_2d_msgs::Twist2D& cmd_vel);

  /**
   * @brief Return true if the robot has travelled far enough or waited long
   * enough
   */
  bool resetAvailable();

  CommandTrend x_trend_, y_trend_, theta_trend_;
  double oscillation_reset_dist_, oscillation_reset_angle_, x_only_threshold_;
  double oscillation_reset_time_;

  // Cached square parameter
  double oscillation_reset_dist_sq_;

  // Saved positions
  geometry_msgs::Pose2D pose_, prev_stationary_pose_;
  // Saved timestamp (steady_clock 等价 rclcpp::Clock)
  std::chrono::steady_clock::time_point prev_reset_time_;
};

}  // namespace dwb_critics
