#pragma once

#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_planner.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

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
      : planner(std::move(planner)) {}

  /// @brief 记录 start/goal/tolerance 并启动规划线程。
  using ExecutionBase::start;
  void start(const geometry_msgs::PoseStamped& start,
             const geometry_msgs::PoseStamped& goal, double tolerance) {
    this->start_pose = start;
    this->goal_pose = goal;
    this->goal_tolerance = tolerance;
    ExecutionBase::start();
  }

  /// @brief 取消: 置 cancel_requested 并透传到 planner 插件 (virtual cancel_requested)。
  void cancel() override {
    cancel_requested = true;
    if (planner) planner->cancel();
    condition.notify_all();
  }

  /// @brief 运行中 in-place 更新目标 (latest-wins update seam, 对标 controller
  /// updatePlan)。线程安全: 把最新 start/goal/tolerance 存入 pending 槽并递增
  /// goal_revision; run() 在 (a) 每次规划前采纳最新 pending 目标, (b) 每次
  /// callPlanner 返回后若 revision 已推进则同一 execution 线程以最新目标重新
  /// 开始 (不 cancel / 不 join / 不新建线程)。同时透传到插件
  /// AbstractPlanner::setNewGoal (CostmapSmacPlanner 用其线程安全 latest goal
  /// revision 在搜索期间检测更新并中止陈旧搜索)。
  /// @return false 表示 execution 已到终态 (run 已返回), 新目标不会被消费,
  /// 调用方应回退到新建 execution。
  virtual bool setNewGoal(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          double tolerance) {
    if (!planner || !planner->setNewGoal(start, goal, tolerance)) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex);
      if (isTerminalState()) return false;
      pending_start = start;
      pending_goal = goal;
      pending_tolerance = tolerance;
      goal_revision.fetch_add(1);
    }
    condition.notify_all();
    return true;
  }

  /// @brief 当前 pending 目标 revision (用于 latest-wins 观察/测试)。
  uint64_t pendingRevision() const { return goal_revision.load(); }

  PlanningState getState() const {
    return static_cast<PlanningState>(state.load());
  }

  /// @brief 最近一次有效路径的代价。
  double getCost() const { return cost; }

  /// @brief 最近一次算出的路径。
  const nav_msgs::Path& getPlan() const { return plan; }

  /// @brief 失败重试次数; -1 无限重试, 0 失败即放弃。
  void setMaxRetries(int retries) { max_retries = retries; }

  /// @brief 规划耐心 (秒); <=0 关闭。
  void setPatience(double seconds) { patience = std::chrono::duration<double>(seconds); }

  bool isPatienceExceeded() const {
    if (patience <= std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - start_time > patience;
  }

protected:
  /// @brief 插件调用钩子: 默认调 abstract 签名; 地图族 execution 覆写为
  /// 带快照的 typed 调用 (在执行层完成快照获取, 插件不接触地图)。
  virtual FailureCode callPlanner(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, double tolerance,
      nav_msgs::Path& plan, double& cost, std::string& message) {
    return planner->makePlan(start, goal, tolerance, plan, cost, message);
  }

  void run() override {
    setState(STARTED, false);
    start_time = std::chrono::steady_clock::now();
    int retries = 0;
    // 当前已采纳的 goal revision (latest-wins): 每次循环采纳最新 pending 目标。
    uint64_t adopted_revision = 0;
    // 本循环使用的 start/goal/tolerance (初始 = start() 传入; 之后随 revision
    // 采纳而切换)。CostmapPlannerExecution::callPlanner 用其构建快照, 故每次
    // 重新规划都基于最新目标。
    ayanami::geometry_msgs::PoseStamped cur_start = this->start_pose;
    ayanami::geometry_msgs::PoseStamped cur_goal = this->goal_pose;
    double cur_tolerance = this->goal_tolerance;
    try {
      while (true) {
        if (cancel_requested) {
          failure = FailureCode::Canceled;
          setState(CANCELED, true);
          return;
        }
        if (should_exit) {
          failure = FailureCode::Canceled;
          setState(STOPPED, true);
          return;
        }
        // latest-wins: 若 setNewGoal 已推入新 revision, 采纳最新 pending 目标。
        // 锁内读取并拷贝, 与 setNewGoal 写入互斥。
        {
          std::lock_guard<std::mutex> lock(goal_mutex);
          if (goal_revision.load() != adopted_revision) {
            cur_start = pending_start;
            cur_goal = pending_goal;
            cur_tolerance = pending_tolerance;
            adopted_revision = goal_revision.load();
            retries = 0;  // 新目标重新计重试预算 (latest-wins)
            message.clear();
          }
        }
        setState(PLANNING, false);

        double cost = 0.0;
        failure = callPlanner(cur_start, cur_goal, cur_tolerance, plan, cost,
                              message);
        this->cost = cost;
        bool success = failure == FailureCode::None;

        // 搜索期间来了新目标 (revision 推进): 不提交本结果, 以同一 execution
        // 线程重新开始规划最新目标 (latest-wins, 无 cancel/join/新线程)。
        {
          std::lock_guard<std::mutex> lock(goal_mutex);
          if (goal_revision.load() != adopted_revision) {
            continue;
          }
        }

        if (cancel_requested && !isPatienceExceeded()) {
          failure = FailureCode::Canceled;
          setState(CANCELED, true);
          return;
        } else if (success) {
          setState(FOUND_PLAN, true);
          return;
        } else if (max_retries > 0 && ++retries > max_retries) {
          if (failure == FailureCode::None) failure = FailureCode::Timeout;
          setState(MAX_RETRIES, true);
          return;
        } else if (isPatienceExceeded()) {
          failure = FailureCode::Timeout;
          setState(PAT_EXCEEDED, true);
          return;
        } else if (max_retries == 0) {
          setState(NO_PLAN_FOUND, true);
          return;
        } else {
          // 重试
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
    } catch (...) {
      failure = FailureCode::InternalError;
      message = "Unknown error occurred";
      setState(INTERNAL_ERROR, true);
    }
  }

  /// @brief 当前状态是否已到终态 (run 已退出 / 不再消费新目标)。
  bool isTerminalState() const {
    const auto s = static_cast<PlanningState>(state.load());
    return s == FOUND_PLAN || s == MAX_RETRIES || s == PAT_EXCEEDED ||
           s == NO_PLAN_FOUND || s == CANCELED || s == STOPPED ||
           s == INTERNAL_ERROR;
  }

protected:
  ayanami::abstract::AbstractPlanner::Ptr planner;
  geometry_msgs::PoseStamped start_pose;
  geometry_msgs::PoseStamped goal_pose;
  double goal_tolerance{0.0};
  nav_msgs::Path plan;
  double cost{0.0};
  int max_retries{-1};
  std::chrono::duration<double> patience{0};
  std::chrono::steady_clock::time_point start_time;

  // ---- latest-wins update seam 状态 (线程安全) ----
  // pending_* 由 setNewGoal 在 goal_mutex 保护下写入, run() 在锁内采纳。
  mutable std::mutex goal_mutex;
  geometry_msgs::PoseStamped pending_start;
  geometry_msgs::PoseStamped pending_goal;
  double pending_tolerance{0.0};
  std::atomic<uint64_t> goal_revision{0};
};

} // namespace components
} // namespace ayanami
