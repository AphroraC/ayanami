#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <cstdint>
#include <memory>

namespace ayanami {
namespace abstract {

/**
 * @brief Progress checker module abstraction (modeled after MBF
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

  /// @brief Interface version of this family, as compiled into the module.
  /// Bump on ANY change to this header's virtual surface: the loader compares
  /// the value a .so reports against the one it was built with and refuses a
  /// mismatch (two modules agreeing on class names but disagreeing on layout is
  /// exactly the failure RTTI cannot catch).
  static constexpr std::uint32_t kApiVersion = 1;
  std::uint32_t apiVersion() const override { return kApiVersion; }


  virtual ~AbstractProgressChecker() = default;

  /// @brief Inject the module logical name + its own config (the module
  /// json pointed to by config_{progress_checker}). Parameters are read
  /// uniformly from here; modules do not read the global config index.
  virtual void configure(const ayanami::ModuleContext& context) {
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