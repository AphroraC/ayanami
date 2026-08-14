#pragma once

#include "ayanami/components/execution_base.hpp"
#include "ayanami/core/abstract_planner.hpp"

#include <chrono>
#include <memory>

namespace ayanami {
namespace components {

/**
 * @brief 全局路径规划执行 (对标 MBF AbstractPlannerExecution, spec §5.1)。
 *
 * 内部状态机枚举独立于 actions::GoalStatus (Ruling #2):
 * INITIALIZED→STARTED→PLANNING→(FOUND_PLAN | MAX_RETRIES | PAT_EXCEEDED |
 * NO_PLAN_FOUND | CANCELED | STOPPED | INTERNAL_ERROR)。
 */
class PlannerExecution : public ExecutionBase {
public:
  using Ptr = std::shared_ptr<PlannerExecution>;

  enum PlanningState {
    INITIALIZED,
    STARTED,
    PLANNING,
    FOUND_PLAN,
    MAX_RETRIES,
    PAT_EXCEEDED,
    NO_PLAN_FOUND,
    CANCELED,
    STOPPED,
    INTERNAL_ERROR,
  };

  explicit PlannerExecution(ayanami::abstract::AbstractPlanner::Ptr planner)
      : planner_(std::move(planner)) {}

  /// @brief 记录 start/goal/tolerance 并启动规划线程。
  using ExecutionBase::start;
  void start(const geometry_msgs::PoseStamped& start,
             const geometry_msgs::PoseStamped& goal, double tolerance) {
    start_ = start;
    goal_ = goal;
    tolerance_ = tolerance;
    ExecutionBase::start();
  }

  PlanningState getState() const {
    return static_cast<PlanningState>(state_.load());
  }

  /// @brief 最近一次有效路径的代价。
  double getCost() const { return cost_; }

  /// @brief 最近一次算出的路径。
  const nav_msgs::Path& getPlan() const { return plan_; }

  /// @brief 失败重试次数; -1 无限重试, 0 失败即放弃。
  void setMaxRetries(int retries) { max_retries_ = retries; }

  /// @brief 规划耐心 (秒); <=0 关闭。
  void setPatience(double seconds) { patience_ = std::chrono::duration<double>(seconds); }

  bool isPatienceExceeded() const {
    if (patience_ <= std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - start_time_ > patience_;
  }

protected:
  void run() override {
    setState(STARTED, false);
    start_time_ = std::chrono::steady_clock::now();
    int retries = 0;
    try {
      while (!should_exit_ && !cancel_) {
        setState(PLANNING, false);

        double cost = 0.0;
        outcome_ = planner_->makePlan(start_, goal_, tolerance_, plan_, cost, message_);
        cost_ = cost;
        bool success = outcome_ == 0;

        if (cancel_ && !isPatienceExceeded()) {
          setState(CANCELED, true);
          return;
        } else if (success) {
          setState(FOUND_PLAN, true);
          return;
        } else if (max_retries_ > 0 && ++retries > max_retries_) {
          setState(MAX_RETRIES, true);
          return;
        } else if (isPatienceExceeded()) {
          setState(PAT_EXCEEDED, true);
          return;
        } else if (max_retries_ == 0) {
          setState(NO_PLAN_FOUND, true);
          return;
        } else {
          // 重试
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
      setState(STOPPED, true);
    } catch (...) {
      message_ = "Unknown error occurred";
      setState(INTERNAL_ERROR, true);
    }
  }

private:
  ayanami::abstract::AbstractPlanner::Ptr planner_;
  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped goal_;
  double tolerance_{0.0};
  nav_msgs::Path plan_;
  double cost_{0.0};
  int max_retries_{-1};
  std::chrono::duration<double> patience_{0};
  std::chrono::steady_clock::time_point start_time_;
};

} // namespace components
} // namespace ayanami
