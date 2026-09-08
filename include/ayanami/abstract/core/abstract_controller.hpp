#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief Local controller plugin abstraction (MBF-style layering).
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

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief Cancel in-progress control (cooperative interrupt; unsupported by default).
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
