#pragma once

#include "ayanami/behavior_tree/resettable_node.hpp"
#include <behaviortree_cpp/decorator_node.h>
#include <spdlog/spdlog.h>


#include <chrono>
#include <functional>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief nav2-style RateController: gates the child execution rate with hz.
 *
 * - hz = -1: execute once, cache the child result, and return the cached
 *   value directly on subsequent ticks;
 * - hz > 0: execute immediately on the first tick; afterwards the child is
 *   re-executed only when at least 1/hz has elapsed since the last
 *   execution; during the cooldown the child's last status is returned.
 *
 * The clock is injectable (steady clock by default) so tests can advance
 * time deterministically.
 */
class RateController : public BT::DecoratorNode,
                    public ResettableNode {
 public:
  using ClockFn = std::function<BT::TimePoint()>;

  RateController(const std::string& name, const BT::NodeConfig& config)
    : RateController(name, config, [] { return BT::TimePoint::clock::now(); }) {}

  RateController(const std::string& name, const BT::NodeConfig& config, ClockFn clock)
    : BT::DecoratorNode(name, config),
      clock_fn(std::move(clock)),
      hz(-1.0),
      last_status(BT::NodeStatus::FAILURE),
      last_time(),
      started(false) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("hz", -1.0, "rate in Hz (-1: run once then cache)"),
      BT::InputPort<uint64_t>("reset_token", 0, "revision that resets the cache"),
    };
  }

  BT::NodeStatus tick() override {
    if (!getInput("hz", hz)) {
      return BT::NodeStatus::FAILURE;
    }
    if (consumeResetToken(*this, "reset_token")) {
      // Token change = semantically "a new upstream artifact must pass
      // through this branch again". Must halt the in-flight child
      // (StatefulActionNode goes through onHalted -> task.cancel), otherwise
      // a still-RUNNING child keeps consuming the old input until completion
      // and the new input is skipped.
      started = false;
      last_status = BT::NodeStatus::FAILURE;
      haltChild();
    }

    if (hz <= 0.0) {
      // Single-shot mode: terminal results (SUCCESS/FAILURE) are cached;
      // a RUNNING async child keeps being re-ticked so onRunning can poll
      // (with hz=-1, ComputePathToPose returns RUNNING after the first tick;
      // if RUNNING were cached without re-ticks, the result would never be
      // ready).
      if (started) {
        if (last_status == BT::NodeStatus::RUNNING) {
          last_status = child_node_->executeTick();
          return last_status;
        }
        return last_status;
      }
      started = true;
      const BT::NodeStatus status = child_node_->executeTick();
      last_status = status;
      return status;
    }

    const BT::TimePoint now = clock_fn();
    if (!started) {
      started = true;
      last_time = now;
      const BT::NodeStatus status = child_node_->executeTick();
      last_status = status;
      return status;
    }

 // nav2 semantics (rate_controller.cpp): an in-flight RUNNING child is
 // polled every tick; the rate only gates "starting a new round" --
 // otherwise, once the planner result is ready it would wait up to a full
 // 1/hz before being consumed, adding a fixed 0-1s delay to every replan.
 // The timer resets when the child returns SUCCESS (failure does not reset,
 // mirroring nav2).
    const auto period = std::chrono::duration<double>(1.0 / hz);
    if (last_status == BT::NodeStatus::RUNNING || now - last_time >= period) {
      const BT::NodeStatus status = child_node_->executeTick();
      last_status = status;
      if (status == BT::NodeStatus::SUCCESS) {
        last_time = now;
      }
      return status;
    }
    return last_status;
  }

  void halt() override {
    BT::DecoratorNode::halt();
    started = false;
    last_status = BT::NodeStatus::FAILURE;
  }

  /// @brief Clear the single-shot cache so the next tick starts a fresh round.
  ///
  /// Called from two places: halt() (BT.CPP v4's haltChildren only reaches
  /// RUNNING children) and the reset_token change path. The reset_token cache
  /// itself is deliberately kept -- it is change-detection state, not round
  /// state. There is no tree-life reset any more: the pilot builds a new tree
  /// per navigation attempt, so nothing crosses between attempts.

 private:
  ClockFn clock_fn;
  double hz;
  BT::NodeStatus last_status;
  BT::TimePoint last_time;
  bool started;
  uint64_t reset_token{0};
};

} // namespace behavior_tree
} // namespace ayanami
