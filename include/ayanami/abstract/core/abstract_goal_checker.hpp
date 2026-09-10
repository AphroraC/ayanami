#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <cstdint>
#include <memory>

namespace ayanami {
namespace abstract {

/**
 * @brief Goal checker module abstraction (modeled after MBF
 * AbstractGoalChecker).
 *
 * Decides whether current_pose has reached goal_pose: position/heading
 * differences within tolerance and velocity below the at-rest threshold.
 * Stateful: reset() should be called when a new goal / replan arrives.
 *
 * This module is the SOLE owner of the arrival contract: the tolerances it
 * reports via getTolerances() are pushed down to the controller by the
 * execution layer (AbstractController::setGoalTolerances), so the controller
 * converges into the same acceptance region instead of assuming its own.
 * The tolerance value types live in ayanami/types.hpp.
 */
class AbstractGoalChecker : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractGoalChecker>;

  /// @brief Interface version of this family, as compiled into the module.
  /// Bump on ANY change to this header's virtual surface: the loader compares
  /// the value a .so reports against the one it was built with and refuses a
  /// mismatch (two modules agreeing on class names but disagreeing on layout is
  /// exactly the failure RTTI cannot catch).
  static constexpr std::uint32_t kApiVersion = 1;
  std::uint32_t apiVersion() const override { return kApiVersion; }


  virtual ~AbstractGoalChecker() = default;

  /// @brief Inject the module logical name + its own config (the module
  /// json pointed to by config_{goal_checker}). Parameters are read
  /// uniformly from here; modules do not read the global config index.
  virtual void configure(const ayanami::ModuleContext& context) {
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