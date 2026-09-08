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
 * @brief Global path planner plugin abstraction (MBF-style layering).
 *
 * Zero map types: the interface contains no map parameters. Map dependency
 * appears only at the map-family layer's injection point
 * configure(PluginContext, map module) (e.g. CostmapPlanner); the host
 * (XXXNavigationServer) owns the map module and injects it.
 *
 * Responsibility table: the plugin only performs path search; failures
 * return an ErrorCode immediately, and retries are decided by the BT.
 */
class AbstractPlanner : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractPlanner>;

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

  /// @brief In-place goal update while running (latest-wins seam; optional).
  virtual bool setNewGoal(const ayanami::geometry_msgs::PoseStamped&,
                          const ayanami::geometry_msgs::PoseStamped&, double) {
    return false;
  }
};

} // namespace abstract
} // namespace ayanami
