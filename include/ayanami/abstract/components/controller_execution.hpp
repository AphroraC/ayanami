#pragma once

#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/abstract/core/abstract_goal_checker.hpp"
#include "ayanami/abstract/core/abstract_progress_checker.hpp"
#include "ayanami/interfaces/callbacks.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace ayanami {
namespace components {

/**
 * @brief 局部控制器执行 (对标 MBF AbstractControllerExecution, spec §5.1)。
 *
 * 按 controller_frequency 循环: progress_checker 判进展 →
 * goal_checker 判到达 → computeVelocityCommands; cmd_vel 经
 * NavigationCallbacks::on_velocity_command 旁路发布。
 *
 * goal/progress checker 由 NavigationServerBase::createActions 注入; checkers
 * 缺失判 InternalError (不回退 controller 自身判定)。setPlan 成功后 reset
 * goal/progress checker 基线并快照 goal 位姿 (新 plan/重规划)。
 *
 * 关键纪律 (spec Global Constraints): 每次发布前 `TwistStamped cmd_vel = {};`
 * 强制清零, 修脏内存历史 bug; publishStopCommand() 也显式清零。
 */
class ControllerExecution : public ExecutionBase {
public:
  using Ptr = std::shared_ptr<ControllerExecution>;

  enum ControllerState {
    INITIALIZED,
    STARTED,
    PLANNING,
    NO_PLAN,
    MAX_RETRIES,
    PAT_EXCEEDED,
    EMPTY_PLAN,
    INVALID_PLAN,
    NO_LOCAL_CMD,
    GOT_LOCAL_CMD,
    ARRIVED_GOAL,
    NO_PROGRESS,
    CANCELED,
    STOPPED,
    INTERNAL_ERROR,
    ROBOT_DISABLED,
  };

  /// @brief 机器人状态供给器 (pose, velocity); 默认返回 identity/零速。
  using RobotStateProvider = std::function<bool(geometry_msgs::PoseStamped&, geometry_msgs::TwistStamped&)>;

  explicit ControllerExecution(ayanami::abstract::AbstractController::Ptr controller)
      : controller(std::move(controller)) {}

  /// @brief 注入机器人位姿/速度源 (Task 10 RobotStateProvider 接入点)。
  void setRobotStateProvider(RobotStateProvider provider) {
    state_provider = std::move(provider);
  }

  /// @brief 设置要跟踪的路径 (可运行中更新, 连续重规划安全)。
  void setNewPlan(const nav_msgs::Path& plan) {
    std::lock_guard<std::mutex> guard(plan_mutex);
    new_plan = true;
    this->plan = plan;
  }

  /// @brief 运行中 in-place 更新路径 + 规划版本 (spec in-place plan update,
  /// 无 cancel_requested/join/new Execution)。下一 cycle 的 run 循环消费 setNewPlan。
  ///
  /// 竞态纪律 (review 修复): 版本分"已应用 (plan_version)"与"待提交
  /// (pending_plan_version)"两轨。本方法只暂存 pending 版本, 不碰 applied;
  /// 终态 execution 直接拒绝 (reject, 返回 false) —— 避免对已经结束的执行暂存
  /// 一个永不消费的版本, 也避免最终结果误报一个 controller->setPlan 从未消费的
  /// 版本。
  bool updatePlan(const nav_msgs::Path& plan, uint64_t plan_version) {
    std::lock_guard<std::mutex> guard(plan_mutex);
    if (isTerminalLocked()) {
      return false;
    }
    new_plan = true;
    this->plan = plan;
    pending_plan_version = plan_version;
    return true;
  }

  /// @brief 覆盖当前规划版本 (初始 goal 应用时用)。applied 与 pending 同步,
  /// 使初次 hasNewPlan 消费后仅回填到同一版本。
  void setPlanVersion(uint64_t plan_version) {
    std::lock_guard<std::mutex> guard(plan_mutex);
    this->plan_version = plan_version;
    pending_plan_version = plan_version;
  }

  /// @brief 当前已应用规划版本 (仅由 controller->setPlan 成功消费后推进)。
  /// finish 报告结果必须用此值: 最终结果只携带真正被 setPlan 消费的版本, 不
  /// 携带仅在途/未消费的 pending 版本。
  uint64_t getPlanVersion() const {
    std::lock_guard<std::mutex> guard(plan_mutex);
    return plan_version;
  }

  bool hasNewPlan() {
    std::lock_guard<std::mutex> guard(plan_mutex);
    return new_plan;
  }

  struct PendingPlan {
    nav_msgs::Path path;
    uint64_t version{0};
  };

  /// @brief 原子消费路径及其对应版本，避免更新线程在两次读取之间插入，
  /// 导致旧路径被错误标记成新版本。
  PendingPlan takeNewPlan() {
    std::lock_guard<std::mutex> guard(plan_mutex);
    new_plan = false;
    return PendingPlan{plan, pending_plan_version};
  }

  void setControllerFrequency(double frequency) {
    if (frequency > 0.0) {
      controller_frequency = frequency;
    }
  }

  double getControllerFrequency() const { return controller_frequency; }

  void setMaxRetries(int retries) { max_retries = retries; }

  void setPatience(double seconds) {
    patience = std::chrono::duration<double>(seconds);
  }

  void setMaximumVelocity(double linear_x, double linear_y,
                          double angular_z) {
    max_linear_x = linear_x;
    max_linear_y = linear_y;
    max_angular_z = angular_z;
  }

  /// @brief 查询当前速度上限 (createActions 从 navigation.max_vel_* 注入)。
  void getMaximumVelocity(double& linear_x, double& linear_y,
                          double& angular_z) const {
    linear_x = max_linear_x;
    linear_y = max_linear_y;
    angular_z = max_angular_z;
  }

  /// @brief 注入目标检查插件 (NavigationServerBase createActions 时 wired)。
  void setGoalChecker(ayanami::abstract::AbstractGoalChecker::Ptr checker) {
    goal_checker = std::move(checker);
  }

  ayanami::abstract::AbstractGoalChecker::Ptr getGoalChecker() const {
    return goal_checker;
  }

  /// @brief 注入进展检查插件 (同上, 运行时消费)。
  void setProgressChecker(
      ayanami::abstract::AbstractProgressChecker::Ptr checker) {
    progress_checker = std::move(checker);
  }

  ayanami::abstract::AbstractProgressChecker::Ptr getProgressChecker() const {
    return progress_checker;
  }

  ControllerState getState() const {
    return static_cast<ControllerState>(state.load());
  }

  bool isPatienceExceeded() const {
    if (patience <= std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - start_time > patience;
  }

  /// @brief 覆写: 清零 TwistStamped 后发布停止命令 (safety cancel_requested)。
  void publishStopCommand() override {
    geometry_msgs::TwistStamped cmd = {};  // 全字段清零
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd);
  }

  /// @brief 取消: 置 cancel_requested → 透传到 controller 插件 (virtual cancel_requested) →
  /// 发布清零停止命令 → notify 唤醒 waitForStateUpdate。
  void cancel() override {
    cancel_requested = true;
    if (controller) controller->cancel();
    publishStopCommand();
    condition.notify_all();
  }

protected:
  /// @brief 插件调用钩子: 默认调 abstract 签名; 地图族 execution 每 cycle
  /// 覆写为 "刷新 local 快照 + 带快照的 typed 调用" (插件不接触地图)。
  virtual FailureCode callController(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity,
      geometry_msgs::TwistStamped& cmd_vel, std::string& message) {
    return controller->computeVelocityCommands(pose, velocity, cmd_vel,
                                               message);
  }

  void run() override {
    start_time = std::chrono::steady_clock::now();
    setState(STARTED, false);

    // 循环周期按 controller_frequency
    const auto period = std::chrono::microseconds(
        static_cast<long long>(1e6 / controller_frequency));

    nav_msgs::Path plan;
    if (!hasNewPlan()) {
      failure = FailureCode::NoValidCommand;
      message = "No plan was provided";
      setState(NO_PLAN, true);
      return;
    }

    // goal/progress checker 必须注入: 缺失为内部错误, 不回退 controller 自身判定。
    if (!goal_checker || !progress_checker) {
      failure = FailureCode::InternalError;
      message = !goal_checker ? "Goal checker is not set"
                                : "Progress checker is not set";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
      return;
    }

    int retries = 0;
    try {
      while (true) {
        if (cancel_requested) {
          failure = FailureCode::Canceled;
          publishStopCommand();
          setState(CANCELED, true);
          return;
        }
        if (should_exit) {
          failure = FailureCode::Canceled;
          publishStopCommand();
          setState(STOPPED, true);
          return;
        }

        if (!safetyCheck()) {
          publishStopCommand();
          std::this_thread::sleep_for(period);
          continue;
        }

        // 动态更新路径 (连续重规划安全)
        if (hasNewPlan()) {
          const PendingPlan pending = takeNewPlan();
          plan = pending.path;
          if (plan.poses.empty()) {
            failure = FailureCode::PathInvalid;
            message = "The path is empty";
            setState(EMPTY_PLAN, true);
            return;
          }
          if (!controller->setPlan(plan)) {
            failure = FailureCode::PathInvalid;
            message = "The controller rejected the path";
            setState(INVALID_PLAN, true);
            return;
          }
          // 新 plan 被 controller->setPlan 真正消费: 此刻才推进 applied
          // 版本 (pending 版本同步回填, 供下一次 in-place 更新暂存新版本)。
          {
            std::lock_guard<std::mutex> guard(plan_mutex);
            plan_version = pending.version;
          }
          // 新 plan 成功应用后: 快照 goal 位姿 + reset goal/progress 基线。
          goal = plan.poses.back();
          goal_checker->reset();
          progress_checker->reset();
        }

        // 获取机器人状态
        geometry_msgs::PoseStamped robot_pose;
        geometry_msgs::TwistStamped robot_velocity;
        if (!state_provider(robot_pose, robot_velocity)) {
          failure = FailureCode::InternalError;
          message = "Could not get the robot pose";
          publishStopCommand();
          setState(INTERNAL_ERROR, true);
          return;
        }

        // 目标检查: goal_checker 是唯一到达判定
        if (goal_checker->isGoalReached(robot_pose, goal, robot_velocity)) {
          failure = FailureCode::None;
          message = "Reached the goal";
          publishStopCommand();
          setState(ARRIVED_GOAL, true);
          return;
        }

        // Only declare the robot stuck after ruling out successful arrival.
        if (!progress_checker->check(robot_pose)) {
          failure = FailureCode::NoProgress;
          message = "No progress made executing the path";
          publishStopCommand();
          setState(NO_PROGRESS, true);
          return;
        }

        setState(PLANNING, false);

        // 关键: 发布前清零 (Global Constraints, 修脏内存 bug)
        geometry_msgs::TwistStamped cmd_vel = {};
        failure = callController(robot_pose, robot_velocity, cmd_vel, message);

        if (failure == FailureCode::None && !validVelocityCommand(cmd_vel)) {
          failure = FailureCode::NoValidCommand;
          message = "Controller produced an invalid velocity command";
        }

        if (failure == FailureCode::None) {
          setState(GOT_LOCAL_CMD, false);
          ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd_vel);
          retries = 0;
        } else if (max_retries > 0 && ++retries > max_retries) {
          if (failure == FailureCode::None) failure = FailureCode::Timeout;
          publishStopCommand();
          setState(MAX_RETRIES, true);
          return;
        } else if (isPatienceExceeded()) {
          failure = FailureCode::Timeout;
          publishStopCommand();
          setState(PAT_EXCEEDED, true);
          return;
        } else {
          setState(NO_LOCAL_CMD, false);
          publishStopCommand();
        }

        std::this_thread::sleep_for(period);
      }
    } catch (...) {
      failure = FailureCode::InternalError;
      message = "Unknown error occurred";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
    }
  }

// 成员 protected: 供派生 (如 CostmapControllerExecution) 覆写钩子
// callController 以注入每周期地图快照 (不用复刻整个 run() 循环)。
protected:
  ayanami::abstract::AbstractController::Ptr controller;
  ayanami::abstract::AbstractGoalChecker::Ptr goal_checker;
  ayanami::abstract::AbstractProgressChecker::Ptr progress_checker;
  ayanami::geometry_msgs::PoseStamped goal;
  RobotStateProvider state_provider = [](geometry_msgs::PoseStamped& pose,
                                          geometry_msgs::TwistStamped& velocity) {
    pose = geometry_msgs::PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
    velocity = geometry_msgs::TwistStamped{};
    return true;
  };

  mutable std::mutex plan_mutex;
  nav_msgs::Path plan;
  bool new_plan{false};
  uint64_t plan_version{0};          ///< 已应用版本 (setPlan 成功消费后推进)。
  uint64_t pending_plan_version{0};  ///< 在途/待应用版本 (updatePlan 暂存)。

  /// @brief 锁内调用: 状态机是否已到终态 (run 循环已 return, 后续不会消费任何
  /// 新 plan)。终态时 in-place updatePlan 一律拒绝, 防止误报版本。
  bool isTerminalLocked() const {
    switch (static_cast<ControllerState>(state.load())) {
      case NO_PLAN:
      case EMPTY_PLAN:
      case INVALID_PLAN:
      case MAX_RETRIES:
      case PAT_EXCEEDED:
      case NO_PROGRESS:
      case ARRIVED_GOAL:
      case CANCELED:
      case STOPPED:
      case INTERNAL_ERROR:
      case ROBOT_DISABLED:
        return true;
      default:
        return false;
    }
  }

  double controller_frequency{20.0};
  double max_linear_x{std::numeric_limits<double>::infinity()};
  double max_linear_y{std::numeric_limits<double>::infinity()};
  double max_angular_z{std::numeric_limits<double>::infinity()};
  int max_retries{-1};
  std::chrono::duration<double> patience{0};
  std::chrono::steady_clock::time_point start_time;

  bool validVelocityCommand(
      const geometry_msgs::TwistStamped& command) const {
    const auto& linear = command.twist.linear;
    const auto& angular = command.twist.angular;
    return std::isfinite(linear.x()) && std::isfinite(linear.y()) &&
           std::isfinite(linear.z()) && std::isfinite(angular.x()) &&
           std::isfinite(angular.y()) && std::isfinite(angular.z()) &&
           std::abs(linear.x()) <= max_linear_x &&
           std::abs(linear.y()) <= max_linear_y &&
           std::abs(angular.z()) <= max_angular_z;
  }
};

} // namespace components
} // namespace ayanami
