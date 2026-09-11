#pragma once

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/runtime_lookup.hpp"
#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <spdlog/spdlog.h>

#include <array>
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
    BT::PortsList own = {
        BT::InputPort<nav_msgs::Path>("path", "path to validate")};
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
      spdlog::warn("[bt] IsPathValid FAIL: server rejected the path (node={})",
                   name());
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
 * @brief Goal update condition: SUCCESS when the goal port changed between
 * ticks, FAILURE otherwise.
 *
 * The first tick only establishes the baseline (init) and is never reported
 * as an update.
 */
class GoalUpdated : public BT::ConditionNode {
 public:
  GoalUpdated(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), initialized(false) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::PoseStamped>(k_goal, "navigation goal") };
  }

  BT::NodeStatus tick() override {
    geometry_msgs::PoseStamped goal;
    if (!getInput(k_goal, goal)) {
      return BT::NodeStatus::FAILURE;
    }
    if (!initialized) {
      initialized = true;
      last_goal = goal;
      return BT::NodeStatus::FAILURE;
    }
    if (goalChanged(goal)) {
      last_goal = goal;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

 private:
  static bool goalChanged(const geometry_msgs::PoseStamped& lhs,
                          const geometry_msgs::PoseStamped& rhs) {
    return !goalEquals(lhs, rhs);
  }

  bool goalChanged(const geometry_msgs::PoseStamped& goal) const {
    return goalChanged(goal, last_goal);
  }

  bool initialized;
  geometry_msgs::PoseStamped last_goal;
};

} // namespace behavior_tree
} // namespace ayanami

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
 * Degraded modes return SUCCESS (progressing assumed): pose unavailable,
 * window not yet filled, or non-positive window. The sample history is
 * process-lifetime per tree instance -- a fresh navigation attempt starts
 * with a fresh assumption of progress.
 */
class IsRobotProgressing : public BT::ConditionNode {
 public:
  IsRobotProgressing(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("distance", 0.1,
                            "minimum displacement over the window [m]"),
      BT::InputPort<double>("window", 5.0, "displacement window [s]"),
    };
  }

  BT::NodeStatus tick() override {
    double distance = 0.1;
    double window = 5.0;
    (void)getInput("distance", distance);
    (void)getInput("window", window);

    // Sample the current pose (best effort: a failed lookup contributes no
    // sample; the window simply ages).
    ayanami::geometry_msgs::PoseStamped pose;
    ayanami::geometry_msgs::TwistStamped velocity;
    if (ayanami::components::RobotStateController::instance().getRobotState(
            pose, velocity)) {
      samples_[write_index_] = Sample{std::chrono::steady_clock::now(),
                                      pose.pose.position.x(),
                                      pose.pose.position.y()};
      write_index_ = (write_index_ + 1) % k_capacity;
      if (sample_count_ < k_capacity) {
        ++sample_count_;
      }
    }

    if (window <= 0.0 || sample_count_ < 2) {
      return BT::NodeStatus::SUCCESS;
    }

    // Judge against the OLDEST sample in the buffer, and only once the
    // window has actually elapsed: a fresh or partially-filled history
    // must read as "progressing" (a stationary robot is declared stuck
    // only after a full window of stillness).
    const Sample& oldest = samples_[(write_index_ + k_capacity -
                                     sample_count_) % k_capacity];
    const Sample& newest = samples_[(write_index_ + k_capacity - 1) %
                                    k_capacity];
    const double elapsed_s =
        std::chrono::duration<double>(newest.time - oldest.time).count();
    if (elapsed_s < window) {
      return BT::NodeStatus::SUCCESS;
    }
    const double displacement =
        std::hypot(newest.x - oldest.x, newest.y - oldest.y);
    if (displacement < distance) {
      spdlog::warn("[bt] IsRobotProgressing FAIL: displacement {:.3f} m < "
                   "{:.3f} m over {:.1f} s (navigation-level oscillation)",
                   displacement, distance, elapsed_s);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

 private:
  struct Sample {
    std::chrono::steady_clock::time_point time;
    double x{0.0};
    double y{0.0};
  };
  static constexpr std::size_t k_capacity = 64;
  std::array<Sample, k_capacity> samples_{};
  std::size_t write_index_{0};
  std::size_t sample_count_{0};
};
