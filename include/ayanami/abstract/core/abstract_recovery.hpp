#pragma once

#include "ayanami/messages/recovery_params.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief Recovery behavior module abstraction (unified execution protocol:
 * start / tick / cancel / finish).
 *
 * Note: recovery-like things are always named recovery. MBF semantics:
 * recovery is a pluggable, cancellable action.
 *
 * RecoveryExecution drives a fixed cycle: start() once -> tick() repeatedly
 * -> finish() once, and acts as the sole velocity command publisher.
 * Modules do not run their own sleep loops nor call
 * NavigationCallbacks::on_velocity_command directly; recoveries needing
 * continuous output such as spin/back_up return the desired velocity for
 * the cycle (optional) from tick(); non-motion recoveries such as map
 * clearing carry no velocity.
 *
 * tick() return contract:
 * - ok=true + status=RUNNING [+ velocity]: recovery in progress;
 * - ok=true + status=SUCCESS: recovery complete;
 * - ok=false + ErrorCode: recovery failed/canceled (CANCELED).
 */
class AbstractRecovery : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractRecovery>;

  /// @brief Interface version of this family, as compiled into the module.
  /// Bump on ANY change to this header's virtual surface: the loader compares
  /// the value a .so reports against the one it was built with and refuses a
  /// mismatch (two modules agreeing on class names but disagreeing on layout is
  /// exactly the failure RTTI cannot catch).
  static constexpr std::uint32_t kApiVersion = 1;
  std::uint32_t apiVersion() const override { return kApiVersion; }


  virtual ~AbstractRecovery() = default;

  /// @brief Nominal period between two tick() calls, pushed by the execution
  /// before start() (seconds).
  ///
  /// The execution owns the cadence, so it is the single source for this value:
  /// a module that integrates velocity over a cycle must take it from here
  /// rather than reading a framework parameter out of its own config file.
  /// Default: no-op, for modules whose logic is period-independent.
  virtual void setCyclePeriod(double seconds) { (void)seconds; }

  /// @brief Called at the start of each run (reset run state; the framework calls it once before execution).
  virtual void start() {}

  /// @brief A single cycle (pure virtual). The framework calls it
  /// repeatedly at a fixed period, driving toward a terminal state.
  /// @param cycle Output parameter: this cycle's status
  ///        (RUNNING/SUCCESS/FAILED/CANCELED) and the suggested velocity
  ///        (optional; non-motion recoveries keep nullopt).
  virtual ResultT<void> tick(RecoveryCycleData& cycle) = 0;

  /// @brief Called at the end of each run (after success/failure/cancel/exception) for cleanup.
  virtual void finish() {}

  /// @brief Inject per-run override params before execution (called by
  /// RecoveryExecution before each run).
  /// Ignored by default; recoveries needing per-run overrides
  /// (spin/back_up/wait) override it: nullopt fields mean keep the
  /// configure-time defaults; interpreting nullopt as 0 is forbidden.
  virtual void setBehaviorParams(const ayanami::RecoveryBehaviorParams& params) {
    (void)params;
  }

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief Cooperatively cancel the in-progress recovery (used by blocking modules to interrupt; unsupported by default).
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
