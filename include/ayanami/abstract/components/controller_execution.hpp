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
      : controller_(std::move(controller)) {}

  /// @brief 注入机器人位姿/速度源 (Task 10 RobotStateProvider 接入点)。
  void setRobotStateProvider(RobotStateProvider provider) {
    state_provider_ = std::move(provider);
  }

  /// @brief 设置要跟踪的路径 (可运行中更新, 连续重规划安全)。
  void setNewPlan(const nav_msgs::Path& plan) {
    std::lock_guard<std::mutex> guard(plan_mutex_);
    new_plan_ = true;
    plan_ = plan;
  }

  bool hasNewPlan() {
    std::lock_guard<std::mutex> guard(plan_mutex_);
    return new_plan_;
  }

  nav_msgs::Path takeNewPlan() {
    std::lock_guard<std::mutex> guard(plan_mutex_);
    new_plan_ = false;
    return plan_;
  }

  void setControllerFrequency(double frequency) {
    if (frequency > 0.0) {
      controller_frequency_ = frequency;
    }
  }

  double getControllerFrequency() const { return controller_frequency_; }

  void setMaxRetries(int retries) { max_retries_ = retries; }

  void setPatience(double seconds) {
    patience_ = std::chrono::duration<double>(seconds);
  }

  /// @brief 注入目标检查插件 (NavigationServerBase createActions 时 wired)。
  void setGoalChecker(ayanami::abstract::AbstractGoalChecker::Ptr checker) {
    goal_checker_ = std::move(checker);
  }

  ayanami::abstract::AbstractGoalChecker::Ptr getGoalChecker() const {
    return goal_checker_;
  }

  /// @brief 注入进展检查插件 (同上, 运行时消费)。
  void setProgressChecker(
      ayanami::abstract::AbstractProgressChecker::Ptr checker) {
    progress_checker_ = std::move(checker);
  }

  ayanami::abstract::AbstractProgressChecker::Ptr getProgressChecker() const {
    return progress_checker_;
  }

  ControllerState getState() const {
    return static_cast<ControllerState>(state_.load());
  }

  bool isPatienceExceeded() const {
    if (patience_ <= std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - start_time_ > patience_;
  }

  /// @brief 覆写: 清零 TwistStamped 后发布停止命令 (safety cancel)。
  void publishStopCommand() override {
    geometry_msgs::TwistStamped cmd = {};  // 全字段清零
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd);
  }

  /// @brief 取消: 置 cancel_ → 透传到 controller 插件 (virtual cancel) →
  /// 发布清零停止命令 → notify 唤醒 waitForStateUpdate。
  void cancel() override {
    cancel_ = true;
    if (controller_) controller_->cancel();
    publishStopCommand();
    condition_.notify_all();
  }

protected:
  void run() override {
    start_time_ = std::chrono::steady_clock::now();
    setState(STARTED, false);

    // 循环周期按 controller_frequency
    const auto period = std::chrono::microseconds(
        static_cast<long long>(1e6 / controller_frequency_));

    nav_msgs::Path plan;
    if (!hasNewPlan()) {
      failure_ = FailureCode::NoValidCommand;
      message_ = "No plan was provided";
      setState(NO_PLAN, true);
      return;
    }

    // goal/progress checker 必须注入: 缺失为内部错误, 不回退 controller 自身判定。
    if (!goal_checker_ || !progress_checker_) {
      failure_ = FailureCode::InternalError;
      message_ = !goal_checker_ ? "Goal checker is not set"
                                : "Progress checker is not set";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
      return;
    }

    int retries = 0;
    try {
      while (true) {
        if (cancel_) {
          failure_ = FailureCode::Canceled;
          publishStopCommand();
          setState(CANCELED, true);
          return;
        }
        if (should_exit_) {
          failure_ = FailureCode::Canceled;
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
          plan = takeNewPlan();
          if (plan.poses.empty()) {
            failure_ = FailureCode::PathInvalid;
            message_ = "The path is empty";
            setState(EMPTY_PLAN, true);
            return;
          }
          if (!controller_->setPlan(plan)) {
            failure_ = FailureCode::PathInvalid;
            message_ = "The controller rejected the path";
            setState(INVALID_PLAN, true);
            return;
          }
          // 新 plan 成功应用后: 快照 goal 位姿 + reset goal/progress 基线。
          goal_ = plan.poses.back();
          goal_checker_->reset();
          progress_checker_->reset();
        }

        // 获取机器人状态
        geometry_msgs::PoseStamped robot_pose;
        geometry_msgs::TwistStamped robot_velocity;
        if (!state_provider_(robot_pose, robot_velocity)) {
          failure_ = FailureCode::InternalError;
          message_ = "Could not get the robot pose";
          publishStopCommand();
          setState(INTERNAL_ERROR, true);
          return;
        }

        // 目标检查: goal_checker 是唯一到达判定
        if (goal_checker_->isGoalReached(robot_pose, goal_, robot_velocity)) {
          failure_ = FailureCode::None;
          message_ = "Reached the goal";
          publishStopCommand();
          setState(ARRIVED_GOAL, true);
          return;
        }

        // Only declare the robot stuck after ruling out successful arrival.
        if (!progress_checker_->check(robot_pose)) {
          failure_ = FailureCode::NoProgress;
          message_ = "No progress made executing the path";
          publishStopCommand();
          setState(NO_PROGRESS, true);
          return;
        }

        setState(PLANNING, false);

        // 关键: 发布前清零 (Global Constraints, 修脏内存 bug)
        geometry_msgs::TwistStamped cmd_vel = {};
        failure_ = controller_->computeVelocityCommands(
            robot_pose, robot_velocity, cmd_vel, message_, goal_checker_);

        if (failure_ == FailureCode::None) {
          setState(GOT_LOCAL_CMD, false);
          ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd_vel);
          retries = 0;
        } else if (max_retries_ > 0 && ++retries > max_retries_) {
          if (failure_ == FailureCode::None) failure_ = FailureCode::Timeout;
          publishStopCommand();
          setState(MAX_RETRIES, true);
          return;
        } else if (isPatienceExceeded()) {
          failure_ = FailureCode::Timeout;
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
      failure_ = FailureCode::InternalError;
      message_ = "Unknown error occurred";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
    }
  }

// 成员 protected: 供派生 (如 CostmapControllerExecution) override run() 复刻
// 循环以实施 per-cycle 锁 (BLOCK #1), 父类 run() 循环逻辑保持 intact。
protected:
  ayanami::abstract::AbstractController::Ptr controller_;
  ayanami::abstract::AbstractGoalChecker::Ptr goal_checker_;
  ayanami::abstract::AbstractProgressChecker::Ptr progress_checker_;
  ayanami::geometry_msgs::PoseStamped goal_;
  RobotStateProvider state_provider_ = [](geometry_msgs::PoseStamped& pose,
                                          geometry_msgs::TwistStamped& velocity) {
    pose = geometry_msgs::PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
    velocity = geometry_msgs::TwistStamped{};
    return true;
  };

  std::mutex plan_mutex_;
  nav_msgs::Path plan_;
  bool new_plan_{false};

  double controller_frequency_{20.0};
  int max_retries_{-1};
  std::chrono::duration<double> patience_{0};
  std::chrono::steady_clock::time_point start_time_;
};

} // namespace components
} // namespace ayanami
