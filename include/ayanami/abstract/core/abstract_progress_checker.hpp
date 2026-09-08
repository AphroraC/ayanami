#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>

namespace ayanami {
namespace abstract {

/**
 * @brief Progress checker plugin abstraction (modeled after MBF
 * AbstractProgressChecker).
 *
 * Tracks whether the robot is making headway: within the configured time
 * window it must move far enough / rotate enough, otherwise it is deemed
 * stuck (no progress). Stateful: reset() should re-baseline on a new goal
 * or before a recovery action.
 */
class AbstractProgressChecker : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractProgressChecker>;

  virtual ~AbstractProgressChecker() = default;

  /// @brief Inject the plugin logical name + its own config (the module
  /// json pointed to by config_{progress_checker}). Parameters are read
  /// uniformly from here; plugins do not read the global config index.
  virtual void configure(const ayanami::PluginContext& context) {
    (void)context;
  }

  /// @brief Reset internal state (new goal / before a recovery action).
  virtual void reset() = 0;

  /// @brief Check whether the current pose shows progress; false = no progress (stuck).
  virtual bool check(
      const ayanami::geometry_msgs::PoseStamped& current_pose) = 0;
};

} // namespace abstract
} // namespace ayanami