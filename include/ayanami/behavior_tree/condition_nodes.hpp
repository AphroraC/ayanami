#pragma once

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/resettable_node.hpp"
#include "ayanami/behavior_tree/runtime_lookup.hpp"
#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief Path validity condition.
 *
 * With a NavigationRuntime injected (resident-tree assembly) this runs the
 * map family's real validation (NavigationRuntime::isPathValid ->
 * CostmapMapContext::isPathValid: in bounds and not lethal/unknown on the
 * global+local overlay); without an injection (pure-blackboard unit tests)
 * it degrades to a non-empty check.
 *
 * Before returning FAILURE the node writes a structured FailureContext
 * (PATH_INVALID) so the surrounding RecoveryNode can classify the failure.
 */
class IsPathValid : public BT::ConditionNode {
public:
  IsPathValid(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config),
    nav_server(lookupNavigationRuntime(config)) {}

  static BT::PortsList providedPorts() {
    auto ports = failureContextPorts();
    BT::PortsList own = {BT::InputPort<nav_msgs::Path>("path", "path to validate")};
    ports.merge(own);
    return ports;
  }

  BT::NodeStatus tick() override {
    nav_msgs::Path path;
    if (!getInput("path", path) || path.poses.empty()) {
      writeContext(ErrorCode::PATH_INVALID, "path port missing or empty");
      return BT::NodeStatus::FAILURE;
    }
    if (nav_server && !nav_server->isPathValid(path)) {
      SPDLOG_LOGGER_WARN(ayanami::btLogger(), "IsPathValid FAIL: server rejected the path (node={})", name());
      writeContext(ErrorCode::PATH_INVALID, "path rejected by map family");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  /// @brief Write a minimal failure context (session/version filled from the
  /// runtime when available) through the declared output port so
  /// RecoveryNode can classify the cause.
  void writeContext(ErrorCode code, const std::string& message) {
    FailureContext ctx;
    ctx.stage = FailureStage::Planner;
    ctx.code = code;
    ctx.message = message;
    if (nav_server) {
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
    }
    setOutput(k_failure_context, ctx);
  }

  components::NavigationRuntime::Ptr nav_server;
};

/**
 * @brief Goal update condition: SUCCESS when the goal changed, FAILURE
 * otherwise.
 *
 * Cross-attempt re-arm on the RESIDENT tree keys off the optional
 * session_id port (production tree binds {session_id}): a session change is
 * exactly an attempt boundary (the pilot opens a session per attempt), and
 * the nav2 GoalUpdatedCondition discipline applies -- the boundary tick
 * only re-establishes the baseline and never reports an update, so the
 * recovery ladder of the new attempt is not short-circuited by the previous
 * attempt's goal. (BT.CPP v4 offers no reliable per-attempt IDLE edge for
 * conditions: ReactiveSequence resets non-running children to IDLE on every
 * tick, and tickRoot resets a terminal root.)
 * Within an attempt the node compares the goal VALUE against the last-seen
 * baseline: an in-flight revision rewrites {goal} on the same session and
 * reads as an update (the recovery ladder yields to the revised task).
 * Trees that bind neither port (hand-built tests) fall back to the plain
 * value comparison.
 */
class GoalUpdated : public BT::ConditionNode {
public:
  GoalUpdated(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::PoseStamped>(k_goal, "navigation goal"),
      // No defaults on purpose: an unbound port must stay absent from
      // input_ports so the detection below engages only when the tree
      // binds it.
      BT::InputPort<uint64_t>(k_session_id, "attempt boundary signal (session change)"),
    };
  }

  BT::NodeStatus tick() override {
    geometry_msgs::PoseStamped goal;
    if (!getInput(k_goal, goal)) {
      return BT::NodeStatus::FAILURE;
    }
    uint64_t session = 0;
    if (config().input_ports.count(k_session_id) > 0 && getInput(k_session_id, session) && session != last_session) {
      // Attempt boundary: re-baseline from the CURRENT goal; no update is
      // reported (nav2 GoalUpdatedCondition IDLE-edge parity).
      last_session = session;
      initialized = true;
      last_goal = goal;
      return BT::NodeStatus::FAILURE;
    }
    if (!initialized) {
      initialized = true;
      last_session = session;
      last_goal = goal;
      return BT::NodeStatus::FAILURE;
    }
    if (goalChanged(goal, last_goal)) {
      last_goal = goal;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  static bool goalChanged(const geometry_msgs::PoseStamped& lhs, const geometry_msgs::PoseStamped& rhs) {
    return !goalEquals(lhs, rhs);
  }

  bool initialized{false};
  uint64_t last_session{0};
  geometry_msgs::PoseStamped last_goal;
};

/**
 * @brief Navigation-level oscillation trigger (MBF oscillation semantics:
 * oscillation_distance / oscillation_timeout).
 *
 * Samples the robot pose every tick into a small ring buffer; FAILS when
 * the displacement over the configured window stays below the configured
 * distance -- the robot is "going nowhere" even though every component
 * individually succeeds (the per-execution progress checker cannot see
 * this planner/controller-level loop). Placed as a ReactiveSequence gate
 * in front of the PipelineSequence: a FAILURE fails the pipeline branch
 * and the surrounding RecoveryNode dispatches the recovery ladder.
 *
 * The sample buffer is sized by construction to cover the configured
 * window: capacity = ceil(window x tick_frequency) + margin (the tick rate
 * comes from navigation.tick_frequency, the loop that drives the tree).
 * A build-time invariant guards the coverage: if the derived capacity
 * exceeds k_max_capacity the node degrades to SUCCESS (progressing
 * assumed) with a one-time warning instead of silently judging on a
 * shorter history than the window requires.
 *
 * Degraded modes return SUCCESS (progressing assumed): pose unavailable,
 * window not yet filled, or non-positive window. The sample history is
 * cleared when the optional reset_token port (production: {goal_revision})
 * changes -- the pilot bumps the revision for every consumed goal, so a
 * fresh navigation attempt starts with a fresh assumption of progress and
 * never inherits the previous attempt's tail stillness (which would
 * false-trigger the recovery ladder on the resident tree). Mid-attempt
 * halts (a dispatched recovery) keep the history: only the pilot-written
 * revision resets it.
 */
class IsRobotProgressing : public BT::ConditionNode, public ayanami::behavior_tree::ResettableNode {
public:
  IsRobotProgressing(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("distance", 0.1, "minimum displacement over the window [m]"),
      BT::InputPort<double>("window", 5.0, "displacement window [s]"),
      BT::InputPort<uint64_t>("reset_token", 0, "revision that clears the sample history"),
    };
  }

  BT::NodeStatus tick() override {
    if (consumeResetToken(*this, "reset_token")) {
      // Attempt boundary (the pilot bumped the revision for a consumed
      // goal): drop the previous attempt's sample history and force the
      // (re)sizing branch below to run again.
      samples_.clear();
      write_index_ = 0;
      sample_count_ = 0;
      configured_window_ = -1.0;
    }

    double distance = 0.1;
    double window = 5.0;
    (void)getInput("distance", distance);
    (void)getInput("window", window);

    if (window != configured_window_ || samples_.empty()) {
      configured_window_ = window;
      // Build-time coverage invariant: the sample buffer is sized to hold
      // window seconds at the tree's tick rate. The rate comes from the
      // pilot-written {tick_frequency} blackboard entry -- read only here,
      // at (re)sizing time, never per tick (no Config access on the hot
      // path). A window demanding more history than k_max_capacity can
      // hold would silently judge on a SHORTER history than the window
      // names; that configuration is rejected into a degraded
      // always-progressing mode with a one-time warning instead.
      double tick_hz = 100.0;
      if (config().blackboard) {
        (void)config().blackboard->get<double>(ayanami::behavior_tree::k_tick_frequency, tick_hz);
      }
      const std::size_t needed = static_cast<std::size_t>(std::ceil(window * tick_hz)) + 2;
      if (window <= 0.0 || needed > k_max_capacity) {
        degraded_ = true;
        samples_.clear();
        sample_count_ = 0;
        write_index_ = 0;
        if (!warned_degraded_) {
          warned_degraded_ = true;
          SPDLOG_LOGGER_WARN(
            ayanami::btLogger(),
            "IsRobotProgressing: window={:.3f}s x tick={:.1f}Hz "
            "exceeds the sample capacity {} -- the gate degrades to "
            "'progressing' and never triggers; lower navigation."
            "oscillation_timeout or raise tick_frequency",
            window,
            tick_hz,
            k_max_capacity);
        }
      } else {
        degraded_ = false;
        samples_.assign(needed, Sample{});
        sample_count_ = 0;
        write_index_ = 0;
      }
    }

    if (degraded_ || window <= 0.0) {
      return BT::NodeStatus::SUCCESS;
    }

    // Sample the current pose (best effort: a failed lookup contributes no
    // sample; the window simply ages).
    ayanami::geometry_msgs::PoseStamped pose;
    ayanami::geometry_msgs::TwistStamped velocity;
    if (ayanami::components::RobotStateController::instance().getRobotState(pose, velocity)) {
      samples_[write_index_] = Sample{std::chrono::steady_clock::now(), pose.pose.position.x(), pose.pose.position.y()};
      write_index_ = (write_index_ + 1) % samples_.size();
      if (sample_count_ < samples_.size()) {
        ++sample_count_;
      }
    }

    if (sample_count_ < 2) {
      return BT::NodeStatus::SUCCESS;
    }

    // Judge against the OLDEST sample in the buffer, and only once the
    // window has actually elapsed: a fresh or partially-filled history
    // must read as "progressing" (a stationary robot is declared stuck
    // only after a full window of stillness).
    const Sample& oldest = samples_[(write_index_ + samples_.size() - sample_count_) % samples_.size()];
    const Sample& newest = samples_[(write_index_ + samples_.size() - 1) % samples_.size()];
    const double elapsed_s = std::chrono::duration<double>(newest.time - oldest.time).count();
    if (elapsed_s < window) {
      return BT::NodeStatus::SUCCESS;
    }
    const double displacement = std::hypot(newest.x - oldest.x, newest.y - oldest.y);
    if (displacement < distance) {
      SPDLOG_LOGGER_WARN(
        ayanami::btLogger(),
        "IsRobotProgressing FAIL: displacement {:.3f} m < "
        "{:.3f} m over {:.1f} s (navigation-level oscillation)",
        displacement,
        distance,
        elapsed_s);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  struct Sample {
    std::chrono::steady_clock::time_point time{};
    double x{0.0};
    double y{0.0};
  };
  /// @brief Hard ceiling on the sample buffer (memory guard); a window
  /// needing more history degrades the node instead of under-sampling.
  static constexpr std::size_t k_max_capacity = 4096;
  std::vector<Sample> samples_;
  std::size_t write_index_{0};
  std::size_t sample_count_{0};
  double configured_window_{-1.0};
  bool degraded_{false};
  bool warned_degraded_{false};
};

}  // namespace behavior_tree
}  // namespace ayanami
