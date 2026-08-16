#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/components/controller_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 局部控制的 costmap 版执行 (spec §5.4, 对标 MBF ControllerCostmapExecution)。
 *
 * 在 ControllerExecution 之上注入 CostmapWrapper 生命周期钩子:
 * - preRun: checkActivate() 幂等激活 costmap;
 * - postRun: checkDeactivate() 幂等停用 costmap;
 * - safetyCheck: costmap_->isCurrent(); 非 current 时发布清零停止命令 (不发速度);
 * - run: 覆写为 per-cycle short lock —— 每 cycle 只在 safetyCheck +
 *   computeVelocityCommands 临界段短持 local wrapper recursive_mutex, 释锁后
 *   emit cmd_vel + sleep 均不持锁 (BLOCK #1, 防 obstacle feed 饿死)。
 */
class CostmapControllerExecution : public ayanami::components::ControllerExecution {
 public:
  using Ptr = std::shared_ptr<CostmapControllerExecution>;

  CostmapControllerExecution(
      const std::string& name,
      ayanami::abstract::AbstractController::Ptr controller,
      costmap_2d::CostmapWrapper::Ptr costmap_wrapper)
      : ControllerExecution(std::move(controller)),
        name_(name),
        costmap_wrapper_(std::move(costmap_wrapper)) {}

  ~CostmapControllerExecution() override = default;

  void preRun() override {
    if (costmap_wrapper_) {
      costmap_wrapper_->checkActivate();
    }
  }

  void postRun() override {
    if (costmap_wrapper_) {
      costmap_wrapper_->checkDeactivate();
    }
  }

  bool safetyCheck() override {
    return !costmap_wrapper_ || costmap_wrapper_->isCurrent();
  }

 protected:
  void run() override {
    // 无 costmap wrapper: 与基类语义一致, 直接跑父类无锁循环。
    if (!costmap_wrapper_) {
      ControllerExecution::run();
      return;
    }

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

        // 动态更新路径 (不持锁)
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

        // 不持锁取机器人状态
        geometry_msgs::PoseStamped robot_pose;
        geometry_msgs::TwistStamped robot_velocity;
        if (!state_provider_(robot_pose, robot_velocity)) {
          message_ = "Could not get the robot pose";
          publishStopCommand();
          setState(INTERNAL_ERROR, true);
          return;
        }

        // 目标检查 (不持锁, 与父类一致)
        if (controller_->isGoalReached(dist_tolerance_, angle_tolerance_)) {
          setState(ARRIVED_GOAL, true);
          return;
        }

        setState(PLANNING, false);

        // 关键 (BLOCK #1): 每 cycle 短持锁 —— 只包 safetyCheck + compute,
        // 让 handleObstaclePointCloud / publishMapsForVisualization 可抢占,
        // 不再整 control loop 持锁饿死它们。
        geometry_msgs::TwistStamped cmd_vel = {};
        {
          std::lock_guard<std::recursive_mutex> lock(costmap_wrapper_->mutex());
          if (!safetyCheck()) {
            publishStopCommand();
            std::this_thread::sleep_for(period);
            continue;
          }
          outcome_ = controller_->computeVelocityCommands(
              robot_pose, robot_velocity, cmd_vel, message_);
        }

        // 释锁后 emit cmd_vel + retry/patience 决策 (均不持锁)
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

        // 不持锁 sleep
        std::this_thread::sleep_for(period);
      }
    } catch (...) {
      message_ = "Unknown error occurred";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
    }
  }

 private:
  std::string name_;
  costmap_2d::CostmapWrapper::Ptr costmap_wrapper_;
};

}  // namespace costmap
}  // namespace ayanami
