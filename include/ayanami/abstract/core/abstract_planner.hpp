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
 * @brief Global path planner module abstraction (MBF-style layering).
 *
 * Zero map types: the interface contains no map parameters. Map dependency
 * appears only at the map-family layer's injection point
 * configure(ModuleContext, map module) (e.g. CostmapPlanner); the host
 * (XXXNavigationServer) owns the map module and injects it.
 *
 * Responsibility table: the module only performs path search; failures
 * return an ErrorCode immediately, and retries are decided by the BT.
 *
 * Pose contract: `start` is ALWAYS a fully resolved, valid pose -- the
 * execution layer (PlannerExecution) substitutes the latest robot pose for
 * an empty start before the module is called. Modules must not interpret
 * sentinel poses themselves. The plan is expected in the start/goal frame
 * (costmap family: the global costmap frame).
 */
class AbstractPlanner : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractPlanner>;

  /// @brief Interface version of this family, as compiled into the module.
  /// Bump on ANY change to this header's virtual surface: the loader compares
  /// the value a .so reports against the one it was built with and refuses a
  /// mismatch (two modules agreeing on class names but disagreeing on layout is
  /// exactly the failure RTTI cannot catch).
  static constexpr std::uint32_t kApiVersion = 1;
  std::uint32_t apiVersion() const override { return kApiVersion; }


  virtual ~AbstractPlanner() = default;

  /// @brief Compute a global path from start to goal (pure virtual).
  /// @param plan Output parameter: on success (ok=true) the path is
  ///        written; on failure the content is undefined.
  /// @return Unified ResultT<void>: failures carry ErrorCode + diagnostics.
  virtual ResultT<void> makePlan(
      const ayanami::geometry_msgs::PoseStamped& start,
      const ayanami::geometry_msgs::PoseStamped& goal,
      double tolerance, ayanami::nav_msgs::Path& plan) = 0;

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief Cancel in-progress planning (cooperative interrupt; unsupported by default).
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
