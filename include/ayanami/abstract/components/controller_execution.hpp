#pragma once

#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/abstract/core/abstract_goal_checker.hpp"
#include "ayanami/abstract/core/abstract_progress_checker.hpp"
#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/types.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>

namespace ayanami {
namespace components {

/**
 * @brief 局部控制器执行 (周期循环)。
 *
 * 按 controller_frequency 循环: progress_checker 判进展 → goal_checker 判
 * 到达 → computeVelocityCommand; cmd_vel 经
 * NavigationCallbacks::on_velocity_command 旁路发布。
 *
 * 机器人状态单一来源: RobotStateController::instance() (位姿 + 速度)。
 *
 * 关键纪律 (Global Constraints): 每次发布前强制清零 TwistStamped, 修脏内存
 * 历史 bug; publishStopCommand() 也显式清零。
 *
 * 失败语义 (职责表): 插件返回失败码或无效命令时, 执行层清零发布并继续循环
 * (execution 内部不做重试计数); 只有 patience > 0 且从 start 起算超过该时长
 * 仍无有效命令时才终态上报 (对标 nav2 controller failure_tolerance), 重试
 * 与恢复决策一律由 BT 决定。
 */
class ControllerExecution : public ExecutionBase {
public:
  using Ptr = std::shared_ptr<ControllerExecution>;

  enum ControllerState {
    INITIALIZED,
    STARTED,
    PLANNING,
    NO_PLAN,
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

  explicit ControllerExecution(ayanami::abstract::AbstractController::Ptr controller)
      : controller(std::move(controller)) {}

  /// @brief 设置要跟踪的路径 (可运行中更新, 连续重规划安全)。
  void setNewPlan(const nav_msgs::Path& plan) {
    std::lock_guard<std::mutex> guard(plan_mutex);
    new_plan = true;
    this->plan = plan;
  }

  /// @brief 提交本 goal 的规划版本 (pending); run 循环消费并 setPlan 成功后
  /// 才进入 applied (getPlanVersion 只回报已应用版本)。
  void setPlanVersion(uint64_t plan_version) {
    std::lock_guard<std::mutex> guard(plan_mutex);
    pending_plan_version = plan_version;
  }

  /// @brief 已应用版本: 仅在 run 循环消费新 plan 且 controller->setPlan
  /// 成功后推进 (in-flight 更新不计入)。
  uint64_t getPlanVersion() const {
    std::lock_guard<std::mutex> guard(plan_mutex);
    return applied_plan_version;
  }

  /// @brief 运行中 in-place 更新路径+版本 (无 cancel/join/new Execution)。
  /// 下一 cycle 的 run 循环消费。
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

  bool hasNewPlan() {
    std::lock_guard<std::mutex> guard(plan_mutex);
    return new_plan;
  }

  /// @brief 原子消费路径+版本, 避免更新线程在两次读取之间插入。
  std::pair<nav_msgs::Path, uint64_t> takeNewPlan() {
    std::lock_guard<std::mutex> guard(plan_mutex);
    new_plan = false;
    return {plan, pending_plan_version};
  }

  void setControllerFrequency(double frequency) {
    if (frequency > 0.0) {
      controller_frequency = frequency;
    }
  }

  double getControllerFrequency() const { return controller_frequency; }

  /// @brief 无有效命令的耐心 (秒, 对标 nav2 failure_tolerance); <=0 关闭
  /// (默认): 失败清零发布并继续循环, 终态只由到达/进展/取消/停止决定。
  void setPatience(double seconds) {
    patience = std::chrono::duration<double>(seconds);
  }

  bool isPatienceExceeded() const {
    if (patience <= std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - start_time > patience;
  }

  void setMaximumVelocity(double linear_x, double linear_y,
                          double angular_z) {
    max_linear_x = linear_x;
    max_linear_y = linear_y;
    max_angular_z = angular_z;
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

  /// @brief 覆写: 清零 TwistStamped 后发布停止命令 (safety cancel)。
  void publishStopCommand() override {
    geometry_msgs::TwistStamped cmd = {};  // 全字段清零
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd);
  }

  /// @brief 取消: 置 cancel_requested → 透传到 controller 插件 →
  /// 发布清零停止命令 → notify 唤醒 waitForStateUpdate。
  void cancel() override {
    cancel_requested = true;
    if (controller) controller->cancel();
    publishStopCommand();
    condition.notify_all();
  }

protected:
  /// @brief 插件调用钩子: 默认调 abstract 签名; 地图族 execution 每 cycle
  /// 覆写为 typed 调用 (插件不接触地图模块)。
  virtual ControllerResultT callController(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity) {
    return controller->computeVelocityCommand(pose, velocity);
  }

  void run() override {
    start_time = std::chrono::steady_clock::now();
    setState(STARTED, false);

    // 循环周期按 controller_frequency
    const auto period = std::chrono::microseconds(
        static_cast<long long>(1e6 / controller_frequency));

    nav_msgs::Path plan;
    if (!hasNewPlan()) {
      error = ErrorCode::NO_VALID_COMMAND;
      message = "No plan was provided";
      setState(NO_PLAN, true);
      return;
    }

    // goal/progress checker 必须注入: 缺失为内部错误, 不回退 controller 自身判定。
    if (!goal_checker || !progress_checker) {
      error = ErrorCode::INTERNAL_ERROR;
      message = !goal_checker ? "Goal checker is not set"
                               : "Progress checker is not set";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
      return;
    }

    try {
      while (true) {
        if (cancel_requested) {
          error = ErrorCode::CANCELED;
          publishStopCommand();
          setState(CANCELED, true);
          return;
        }
        if (should_exit) {
          error = ErrorCode::CANCELED;
          publishStopCommand();
          setState(STOPPED, true);
          return;
        }

        if (!safetyCheck()) {
          publishStopCommand();
          sleepInterruptibly(period);
          continue;
        }

        // 动态更新路径 (连续重规划安全)
        if (hasNewPlan()) {
          uint64_t plan_version = 0;
          std::tie(plan, plan_version) = takeNewPlan();
          if (plan.poses.empty()) {
            error = ErrorCode::PATH_INVALID;
            message = "The path is empty";
            setState(EMPTY_PLAN, true);
            return;
          }
          if (!controller->setPlan(plan)) {
            error = ErrorCode::PATH_INVALID;
            message = "The controller rejected the path";
            setState(INVALID_PLAN, true);
            return;
          }
          // setPlan 成功后版本才进入 applied (结果只报已应用版本)。
          {
            std::lock_guard<std::mutex> guard(plan_mutex);
            applied_plan_version = plan_version;
          }
          // 新 plan 成功应用后: 快照 goal 位姿 + reset goal/progress 基线。
          goal = plan.poses.back();
          goal_checker->reset();
          progress_checker->reset();
        }

        // 机器人状态单一来源: 全局 RobotStateController (位姿 + 最新速度)。
        geometry_msgs::PoseStamped robot_pose;
        geometry_msgs::TwistStamped robot_velocity;
        if (!RobotStateController::instance().getRobotState(
                robot_pose, robot_velocity)) {
          error = ErrorCode::TRANSFORM_UNAVAILABLE;
          message = "Could not get the robot pose";
          publishStopCommand();
          setState(INTERNAL_ERROR, true);
          return;
        }

        // 目标检查: goal_checker 是唯一到达判定
        if (goal_checker->isGoalReached(robot_pose, goal, robot_velocity)) {
          error = ErrorCode::NONE;
          message = "Reached the goal";
          publishStopCommand();
          setState(ARRIVED_GOAL, true);
          return;
        }

        // Only declare the robot stuck after ruling out successful arrival.
        if (!progress_checker->check(robot_pose)) {
          error = ErrorCode::NO_PROGRESS;
          message = "No progress made executing the path";
          publishStopCommand();
          setState(NO_PROGRESS, true);
          return;
        }

        setState(PLANNING, false);

        const ControllerResultT result =
            callController(robot_pose, robot_velocity);

        // 插件调用期间发生取消/停止: 不发布本轮命令 (防止 recovery 启动瞬间
        // 漏出一条旧速度命令), 下一轮循环顶部统一结算 CANCELED/STOPPED。
        if (cancel_requested || should_exit) {
          continue;
        }

        if (result.ok) {
          // 关键: 发布前清零 (Global Constraints, 修脏内存 bug)
          geometry_msgs::TwistStamped cmd_vel = {};
          cmd_vel = result.data.velocity;
          if (!validVelocityCommand(cmd_vel)) {
            error = ErrorCode::NO_VALID_COMMAND;
            message = "Controller produced an invalid velocity command";
            publishStopCommand();
          } else {
            error = ErrorCode::NONE;
            message = result.message;
            setState(GOT_LOCAL_CMD, false);
            ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd_vel);
          }
        } else {
          error = result.error;
          message = result.message;
        }

        if (error != ErrorCode::NONE) {
          if (isPatienceExceeded()) {
            error = ErrorCode::TIMEOUT;
            publishStopCommand();
            setState(INTERNAL_ERROR, true);
            return;
          }
          setState(NO_LOCAL_CMD, false);
          publishStopCommand();
        }

        sleepInterruptibly(period);
      }
    } catch (...) {
      error = ErrorCode::INTERNAL_ERROR;
      message = "Unknown error occurred";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
    }
  }

// 成员 protected: 供派生 (如 CostmapControllerExecution) 覆写钩子
// callController 以注入地图访问 (不用复刻整个 run 循环)。
protected:
  ayanami::abstract::AbstractController::Ptr controller;
  ayanami::abstract::AbstractGoalChecker::Ptr goal_checker;
  ayanami::abstract::AbstractProgressChecker::Ptr progress_checker;
  ayanami::geometry_msgs::PoseStamped goal;

  mutable std::mutex plan_mutex;
  nav_msgs::Path plan;
  bool new_plan{false};
  uint64_t pending_plan_version{0};
  uint64_t applied_plan_version{0};

  /// @brief 锁内调用: 状态机是否已到终态 (run 循环已 return)。
  bool isTerminalLocked() const {
    switch (static_cast<ControllerState>(state.load())) {
      case NO_PLAN:
      case EMPTY_PLAN:
      case INVALID_PLAN:
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

  /// @brief 周期等待; cancel/stop 立即打断 (不等满周期)。
  void sleepInterruptibly(std::chrono::microseconds period) {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait_for(lock, period, [this] {
      return cancel_requested.load() || should_exit.load();
    });
  }

  double controller_frequency{20.0};
  double max_linear_x{std::numeric_limits<double>::infinity()};
  double max_linear_y{std::numeric_limits<double>::infinity()};
  double max_angular_z{std::numeric_limits<double>::infinity()};
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
