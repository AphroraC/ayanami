#pragma once

#include "ayanami/components/execution_base.hpp"
#include "ayanami/core/abstract_controller.hpp"
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
 * 按 controller_frequency 循环 computeVelocityCommands + isGoalReached;
 * cmd_vel 经 NavigationCallbacks::on_velocity_command 旁路发布。
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

  void setTolerances(double dist, double angle) {
    dist_tolerance_ = dist;
    angle_tolerance_ = angle;
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
      setState(NO_PLAN, true);
      return;
    }

    int retries = 0;
    try {
      while (true) {
        if (cancel_) {
          publishStopCommand();
          setState(CANCELED, true);
          return;
        }
        if (should_exit_) {
          publishStopCommand();
          setState(STOPPED, true);
          return;
        }

        if (!safetyCheck()) {
          publishStopCommand();
          std::this_thread::sleep_for(period);
          continue;
        }

        // 动态更新路径
        if (hasNewPlan()) {
          plan = takeNewPlan();
          if (plan.poses.empty()) {
            setState(EMPTY_PLAN, true);
            return;
          }
          if (!controller_->setPlan(plan)) {
            setState(INVALID_PLAN, true);
            return;
          }
        }

        // 获取机器人状态
        geometry_msgs::PoseStamped robot_pose;
        geometry_msgs::TwistStamped robot_velocity;
        if (!state_provider_(robot_pose, robot_velocity)) {
          message_ = "Could not get the robot pose";
          publishStopCommand();
          setState(INTERNAL_ERROR, true);
          return;
        }

        // 检查是否到达目标
        if (controller_->isGoalReached(dist_tolerance_, angle_tolerance_)) {
          setState(ARRIVED_GOAL, true);
          return;
        }

        setState(PLANNING, false);

        // 关键: 发布前清零 (Global Constraints, 修脏内存 bug)
        geometry_msgs::TwistStamped cmd_vel = {};
        outcome_ = controller_->computeVelocityCommands(robot_pose, robot_velocity,
                                                        cmd_vel, message_);

        if (outcome_ == 0) {
          setState(GOT_LOCAL_CMD, false);
          ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd_vel);
          retries = 0;
        } else if (max_retries_ > 0 && ++retries > max_retries_) {
          publishStopCommand();
          setState(MAX_RETRIES, true);
          return;
        } else if (isPatienceExceeded()) {
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
      message_ = "Unknown error occurred";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
    }
  }

private:
  ayanami::abstract::AbstractController::Ptr controller_;
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
  double dist_tolerance_{0.1};
  double angle_tolerance_{0.1};
  int max_retries_{-1};
  std::chrono::duration<double> patience_{0};
  std::chrono::steady_clock::time_point start_time_;
};

} // namespace components
} // namespace ayanami
