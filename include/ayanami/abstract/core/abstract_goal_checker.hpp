#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>

namespace ayanami {
namespace abstract {

/**
 * @brief Pose tolerance (semantics equivalent to geometry_msgs/
 * PoseWithCovariance).
 *
 * position: xy planar position tolerance (m); angle: yaw angle tolerance
 * (rad).
 */
struct PoseTolerance {
  double position = 0.10;
  double angle = 0.20;
};

/**
 * @brief Velocity tolerance (body frame, stop-decision boundary).
 *
 * linear: linear velocity tolerance (m/s); angular: angular velocity
 * tolerance (rad/s).
 */
struct TwistTolerance {
  double linear = 0.10;
  double angular = 0.10;
};

/**
 * @brief Combined goal-check tolerances (getTolerances return value).
 */
struct GoalCheckerTolerances {
  PoseTolerance pose;
  TwistTolerance velocity;
};

/**
 * @brief Goal checker plugin abstraction (modeled after MBF
 * AbstractGoalChecker).
 *
 * Decides whether current_pose has reached goal_pose: position/heading
 * differences within tolerance and velocity below the at-rest threshold.
 * Stateful: reset() should be called when a new goal / replan arrives.
 */
class AbstractGoalChecker : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractGoalChecker>;

  virtual ~AbstractGoalChecker() = default;

  /// @brief Inject the plugin logical name + its own config (the module
  /// json pointed to by config_{goal_checker}). Parameters are read
  /// uniformly from here; plugins do not read the global config index.
  virtual void configure(const ayanami::PluginContext& context) {
    (void)context;
  }

  /// @brief Reset internal state (new goal / replan).
  virtual void reset() = 0;

  /// @brief Decide whether the goal pose has been reached.
  virtual bool isGoalReached(
      const ayanami::geometry_msgs::PoseStamped& current_pose,
      const ayanami::geometry_msgs::PoseStamped& goal_pose,
      const ayanami::geometry_msgs::TwistStamped& current_velocity) = 0;

  /// @brief Currently effective pose/velocity tolerances.
  virtual GoalCheckerTolerances getTolerances() const = 0;
};

} // namespace abstract
} // namespace ayanami