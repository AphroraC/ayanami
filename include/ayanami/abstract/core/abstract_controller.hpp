#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief Local controller module abstraction (MBF-style layering).
 *
 * Takes the current pose/velocity and outputs a single-cycle velocity
 * command; the path to follow is received via setPlan.
 * Arrival is decided by the execution layer via goal_checker (the
 * controller itself does not decide arrival).
 * Zero map types: the interface contains no map parameters.
 */
class AbstractController : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractController>;

  /// @brief Interface version of this family, as compiled into the module.
  /// Bump on ANY change to this header's virtual surface: the loader compares
  /// the value a .so reports against the one it was built with and refuses a
  /// mismatch (two modules agreeing on class names but disagreeing on layout is
  /// exactly the failure RTTI cannot catch).
  static constexpr std::uint32_t kApiVersion = 1;
  std::uint32_t apiVersion() const override { return kApiVersion; }


  virtual ~AbstractController() = default;

  /// @brief Compute a single-cycle velocity command (pure virtual).
  /// @param cmd Output parameter: on success (ok=true) the velocity command
  ///        is written; on failure the content is undefined and the caller
  ///        must not publish it.
  /// @return Unified ResultT<void>: failures carry ErrorCode + diagnostics
  ///         (decisions look only at ErrorCode).
  virtual ResultT<void> computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped& pose,
      const ayanami::geometry_msgs::TwistStamped& velocity,
      ayanami::geometry_msgs::TwistStamped& cmd) = 0;

  /// @brief Set the path to follow.
  virtual bool setPlan(const ayanami::nav_msgs::Path& plan) = 0;

  /// @brief Arrival contract to converge into, pushed by the execution layer
  /// from the goal checker (the sole owner of the arrival decision).
  ///
  /// The controller may keep its own preference for how tightly to track the
  /// goal, but it must never stop outside this tolerance -- otherwise a
  /// tolerance tighter than the controller's own preference would be
  /// unreachable and the module would hold station outside the acceptance
  /// region forever (no arrival, no failure, just patience timeout).
  ///
  /// Optional by design: the execution layer calls this before every run when
  /// a checker is present. A controller that never receives it must fall back
  /// to its configure-time parameters (standalone/test use).
  virtual void setGoalTolerances(const ayanami::PoseTolerance&) {}

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief Cancel in-progress control (cooperative interrupt; unsupported by default).
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
