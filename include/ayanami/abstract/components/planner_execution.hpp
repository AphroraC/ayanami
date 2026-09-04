#pragma once

#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/types.hpp"

#include <cstdint>
#include <memory>
#include <mutex>

namespace ayanami {
namespace components {

/**
 * @brief 全局路径规划执行 (单次执行, 对标 MBF AbstractPlannerExecution)。
 *
 * 职责表: 插件只做路径搜索; 失败立即映射为 ErrorCode 终态, 重试由 BT
 * (RecoveryNode) 决定 —— execution 层不做重试/耐心循环。
 *
 * latest-wins 目标 seam: setNewGoal 把最新 start/goal/tolerance 存入 pending
 * 槽; run() 在每次插件调用前采纳最新 pending 目标, 插件调用返回后若
 * revision 已推进则以最新目标重新开始 (不 cancel / 不 join / 不新建线程)。
 *
 * 内部状态机独立于 actions::GoalStatus (Ruling #2):
 * INITIALIZED→STARTED→PLANNING→(FOUND_PLAN | NO_PLAN_FOUND | CANCELED |
 * STOPPED | INTERNAL_ERROR)。
 */
class PlannerExecution : public ExecutionBase {
public:
  using Ptr = std::shared_ptr<PlannerExecution>;

  enum PlanningState {
    INITIALIZED,
    STARTED,
    PLANNING,
    FOUND_PLAN,
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

  /// @brief 取消: 置 cancel_requested 并透传到 planner 插件 (协作式)。
  void cancel() override {
    cancel_requested = true;
    if (planner) planner->cancel();
    condition.notify_all();
  }

  /// @brief 运行中 in-place 更新目标 (latest-wins update seam)。
  /// @return false 表示 execution 已到终态或插件不支持, 调用方应回退到
  /// 新建 execution。
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

  /// @brief 最近一次算出的路径。
  const nav_msgs::Path& getPlan() const { return plan; }

protected:
  /// @brief 插件调用钩子: 默认调 abstract 签名; 地图族 execution 覆写为
  /// typed 调用 (执行层完成地图访问, 插件不接触地图模块)。
  virtual PlannerResultT callPlanner(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, double tolerance) {
    return planner->makePlan(start, goal, tolerance);
  }

  void run() override {
    setState(STARTED, false);
    // 当前已采纳的 goal revision (latest-wins): 每次循环采纳最新 pending 目标。
    uint64_t adopted_revision = 0;
    ayanami::geometry_msgs::PoseStamped cur_start = this->start_pose;
    ayanami::geometry_msgs::PoseStamped cur_goal = this->goal_pose;
    double cur_tolerance = this->goal_tolerance;
    try {
      while (true) {
        if (cancel_requested) {
          error = ErrorCode::CANCELED;
          setState(CANCELED, true);
          return;
        }
        if (should_exit) {
          error = ErrorCode::CANCELED;
          setState(STOPPED, true);
          return;
        }
        // latest-wins: 若 setNewGoal 已推入新 revision, 采纳最新 pending 目标。
        {
          std::lock_guard<std::mutex> lock(goal_mutex);
          if (goal_revision.load() != adopted_revision) {
            cur_start = pending_start;
            cur_goal = pending_goal;
            cur_tolerance = pending_tolerance;
            adopted_revision = goal_revision.load();
          }
        }
        setState(PLANNING, false);

        const PlannerResultT result =
            callPlanner(cur_start, cur_goal, cur_tolerance);

        // 插件调用期间来了新目标 (revision 推进): 不提交本结果, 以同一
        // execution 线程重新规划最新目标 (latest-wins, 无 cancel/新线程)。
        {
          std::lock_guard<std::mutex> lock(goal_mutex);
          if (goal_revision.load() != adopted_revision) {
            continue;
          }
        }

        if (cancel_requested) {
          error = ErrorCode::CANCELED;
          setState(CANCELED, true);
          return;
        }
        if (result.ok) {
          plan = result.data;
          error = ErrorCode::NONE;
          message = result.message;
          setState(FOUND_PLAN, true);
          return;
        }
        // 失败立即返回错误码 (重试由 BT 决定)。
        error = result.error;
        message = result.message;
        setState(NO_PLAN_FOUND, true);
        return;
      }
    } catch (...) {
      error = ErrorCode::INTERNAL_ERROR;
      message = "Unknown error occurred";
      setState(INTERNAL_ERROR, true);
    }
  }

  /// @brief 当前状态是否已到终态 (run 已退出 / 不再消费新目标)。
  bool isTerminalState() const {
    const auto s = static_cast<PlanningState>(state.load());
    return s == FOUND_PLAN || s == NO_PLAN_FOUND || s == CANCELED ||
           s == STOPPED || s == INTERNAL_ERROR;
  }

protected:
  ayanami::abstract::AbstractPlanner::Ptr planner;
  geometry_msgs::PoseStamped start_pose;
  geometry_msgs::PoseStamped goal_pose;
  double goal_tolerance{0.0};
  nav_msgs::Path plan;

  // ---- latest-wins update seam 状态 (线程安全) ----
  mutable std::mutex goal_mutex;
  geometry_msgs::PoseStamped pending_start;
  geometry_msgs::PoseStamped pending_goal;
  double pending_tolerance{0.0};
  std::atomic<uint64_t> goal_revision{0};
};

} // namespace components
} // namespace ayanami
