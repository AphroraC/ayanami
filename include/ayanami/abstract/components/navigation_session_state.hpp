#pragma once

#include <atomic>
#include <cstdint>

namespace ayanami {
namespace components {

/**
 * @brief Single source of truth for navigation session/version state
 * (BehaviorTreePilot and NavigationServerBase share the same object).
 *
 * Background: the pilot originally held its own session_id/plan_version
 * atomics and the server held another; two counters incrementing
 * independently diverge -- the blackboard session and the actual goal's
 * session disagree. This object is the single source: the pilot calls
 * startNewSession() in its tick thread's applyPendingGoal (session+1 and
 * plan_version reset to zero), and the server's
 * currentSessionId/currentPlanVersion both delegate here, so both see the
 * same session number.
 *
 * Thread safety: session/plan_version use std::atomic and are readable
 * from any thread; startNewSession / incrementPlanVersion /
 * sessionIsCurrent / planVersionIsCurrent are all lock-free.
 */
class NavigationSessionState {
 public:
  NavigationSessionState() = default;

  /// @brief Current session number (initially 0; +1 on each startNewSession).
  uint64_t currentSessionId() const { return session_id.load(); }

  /// @brief Current plan version (initially 0; reset to zero by startNewSession).
  uint64_t currentPlanVersion() const { return plan_version.load(); }

  /// @brief Start a new session: session_id+1 and plan_version reset to zero (the start of each navigation round).
  void startNewSession() {
    session_id.fetch_add(1, std::memory_order_acq_rel);
    plan_version.store(0, std::memory_order_release);
  }

  /// @brief Increment the plan version (when smoothing/replanning produces a new path).
  void incrementPlanVersion() {
    plan_version.fetch_add(1, std::memory_order_acq_rel);
  }

  /// @brief Whether the result's session still belongs to the current session.
  bool sessionIsCurrent(uint64_t session_id) const {
    return session_id == this->session_id.load();
  }

  /// @brief Whether the result's plan_version is still the current version.
  bool planVersionIsCurrent(uint64_t plan_version) const {
    return plan_version == this->plan_version.load();
  }

 private:
  std::atomic<uint64_t> session_id{0};
  std::atomic<uint64_t> plan_version{0};
};

}  // namespace components
}  // namespace ayanami