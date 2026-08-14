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
      clock_fn_(std::move(clock)),
      hz_(-1.0),
      last_status_(BT::NodeStatus::FAILURE),
      last_time_(),
      started_(false) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("hz", -1.0, "rate in Hz (-1: run once then cache)"),
    };
  }

  BT::NodeStatus tick() override {
    if (!getInput("hz", hz_)) {
      return BT::NodeStatus::FAILURE;
    }

    if (hz_ <= 0.0) {
      // 单次模式: 执行一次后缓存
      if (started_) {
        return last_status_;
      }
      started_ = true;
      const BT::NodeStatus status = child_node_->executeTick();
      last_status_ = status;
      return status;
    }

    const BT::TimePoint now = clock_fn_();
    if (!started_) {
      started_ = true;
      last_time_ = now;
      const BT::NodeStatus status = child_node_->executeTick();
      last_status_ = status;
      return status;
    }

    // 限频模式: 距上次执行 >= 1/hz 才重新执行
    const auto period = std::chrono::duration<double>(1.0 / hz_);
    if (now - last_time_ >= period) {
      last_time_ = now;
      const BT::NodeStatus status = child_node_->executeTick();
      last_status_ = status;
      return status;
    }
    return last_status_;
  }

  void halt() override {
    BT::DecoratorNode::halt();
    started_ = false;
    last_status_ = BT::NodeStatus::FAILURE;
  }

 private:
  ClockFn clock_fn_;
  double hz_;
  BT::NodeStatus last_status_;
  BT::TimePoint last_time_;
  bool started_;
};

} // namespace behavior_tree
} // namespace ayanami
