#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/abstract/core/abstract_goal_checker.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief Simple goal checker module (modeled on MBF SimpleGoalChecker).
 *
 * Pure C++ (zero ROS): xy distance of pose.position and relative rotation angle of pose.angle within tolerance,
 * and linear velocity < velocity.linear and angular velocity < velocity.angular -> arrived.
 * Tolerances are configured via the simple_goal_checker section of simple_goal_checker.json
 * (injected from PluginContext at configure time).
 */
class SimpleGoalChecker final : public ayanami::abstract::AbstractGoalChecker {
 public:
  SimpleGoalChecker() = default;

  void configure(const ayanami::PluginContext& context) override;

  void reset() override;

  bool isGoalReached(const ayanami::geometry_msgs::PoseStamped& current_pose,
                     const ayanami::geometry_msgs::PoseStamped& goal_pose,
                     const ayanami::geometry_msgs::TwistStamped& current_velocity) override;

  ayanami::abstract::GoalCheckerTolerances getTolerances() const override;

 private:
  struct SimpleGoalCheckerParam {
    double position_tolerance = 0.10;
    double angle_tolerance = 0.20;
    double linear_tolerance = 0.10;
    double angular_tolerance = 0.10;
  };

  SimpleGoalCheckerParam param;

  std::atomic<bool> initialized{false};
};

}  // namespace modules
}  // namespace ayanami
