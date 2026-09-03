#pragma once

#include <behaviortree_cpp/decorator_node.h>

#include <chrono>
#include <functional>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief nav2 风格 RateController (spec §6.4): 以 hz 门控 child 执行频率。
 *
 * - hz = -1: 单次执行后缓存 child 结果, 后续 tick 直接返回缓存值;
 * - hz > 0: 首 tick 立即执行, 之后只有距上次执行 >= 1/hz 才重新执行 child,
 *   冷却期间返回 child 上次状态。
 *
 * 时钟可注入 (默认 steady 时钟), 便于测试确定性地推进时间。
 */
class RateController : public BT::DecoratorNode {
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
    uint64_t token = 0;
    (void)getInput("reset_token", token);
    if (token != reset_token) {
      reset_token = token;
      started = false;
      last_status = BT::NodeStatus::FAILURE;
    }

    if (hz <= 0.0) {
      // 单次模式: 终端结果 (SUCCESS/FAILURE) 缓存, RUNNING 的异步 child 持续
      // re-tick 让 onRunning 轮询 (Task 12 stage C: hz=-1 下 ComputePathToPose
      // 首次 tick 后返回 RUNNING, 若缓存 RUNNING 不再 re-tick, 结果永不就绪)。
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

    // 限频模式: 距上次执行 >= 1/hz 才重新执行
    const auto period = std::chrono::duration<double>(1.0 / hz);
    if (now - last_time >= period) {
      last_time = now;
      const BT::NodeStatus status = child_node_->executeTick();
      last_status = status;
      return status;
    }
    return last_status;
  }

  void halt() override {
    BT::DecoratorNode::halt();
    started = false;
    last_status = BT::NodeStatus::FAILURE;
  }

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
