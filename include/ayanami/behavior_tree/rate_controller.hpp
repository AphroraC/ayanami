#pragma once

#include <behaviortree_cpp/decorator_node.h>

#include <chrono>
#include <functional>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief nav2 风格 RateController : 以 hz 门控 child 执行频率。
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
      // 令牌变化 = 语义上"一个新的上游产物需要重新经过本分支"。必须 halt
      // 在飞 child (StatefulActionNode 走 onHalted → task.cancel), 否则仍在
      // RUNNING 的 child 只会继续消费旧输入直至完成, 新输入被跳过。
      reset_token = token;
      started = false;
      last_status = BT::NodeStatus::FAILURE;
      haltChild();
    }

    if (hz <= 0.0) {
      // 单次模式: 终端结果 (SUCCESS/FAILURE) 缓存, RUNNING 的异步 child 持续
      // re-tick 让 onRunning 轮询 (hz=-1 下 ComputePathToPose
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

 // nav2 语义 (rate_controller.cpp): 在飞的 RUNNING child 每个 tick 都被
 // 轮询, 速率只限制"启动新一轮" —— 否则 planner 结果就绪后最长要等满
 // 1/hz 才被消费, 每次重规划引入 0~1s 固定延迟。
 // 计时器在 child SUCCESS 时复位 (failure 不重置, 对标 nav2)。
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

  /// @brief 树生命终端后的逻辑重置 (pilot 调 resetNavigationTreeState)。
  ///
  /// BT.CPP v4 的 ControlNode::haltChildren 只对 status==RUNNING 的 child 调
  /// haltNode, 终态 (SUCCESS/FAILURE 缓存) 的子树不会被 halt —— 常驻树模型下
  /// 缓存标志必须由 pilot 在树生命结束时显式重置, 否则 planner/smoother 分支
  /// 永不重新执行 (恢复风暴 bug 根因)。reset_token 缓存保留 (变更检测语义)。
  void resetState() {
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
