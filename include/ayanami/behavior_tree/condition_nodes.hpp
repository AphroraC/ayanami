#pragma once

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <cstdint>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief 路径有效性条件: 黑板路径存在且 poses 非空 -> SUCCESS。
 */
class IsPathValid : public BT::ConditionNode {
 public:
  IsPathValid(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<nav_msgs::Path>("path", "path to validate") };
  }

  BT::NodeStatus tick() override {
    nav_msgs::Path path;
    if (!getInput("path", path)) {
      return BT::NodeStatus::FAILURE;
    }
    return path.poses.empty() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

/**
 * @brief 到达目标条件: 机器人位姿与 goal 距离 <= tolerance -> SUCCESS。
 */
class IsGoalReached : public BT::ConditionNode {
 public:
  IsGoalReached(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), tolerance_(0.5) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::PoseStamped>("goal", "navigation goal"),
      BT::InputPort<geometry_msgs::PoseStamped>("robot_pose", "current robot pose"),
      BT::InputPort<double>("tolerance", 0.5, "goal tolerance in meters"),
    };
  }

  BT::NodeStatus tick() override {
    geometry_msgs::PoseStamped goal, robot_pose;
    if (!getInput("goal", goal) || !getInput("robot_pose", robot_pose) ||
        !getInput("tolerance", tolerance_)) {
      return BT::NodeStatus::FAILURE;
    }
    const double dist =
        (goal.pose.translation() - robot_pose.pose.translation()).norm();
    return dist <= tolerance_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

 private:
  double tolerance_;
};

/**
 * @brief 距目标仍远条件: 机器人位姿与 goal 距离 > threshold -> SUCCESS。
 */
class IsFarFromGoal : public BT::ConditionNode {
 public:
  IsFarFromGoal(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), threshold_(0.5) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::PoseStamped>("goal", "navigation goal"),
      BT::InputPort<geometry_msgs::PoseStamped>("robot_pose", "current robot pose"),
      BT::InputPort<double>("threshold", 0.5, "distance threshold in meters"),
    };
  }

  BT::NodeStatus tick() override {
    geometry_msgs::PoseStamped goal, robot_pose;
    if (!getInput("goal", goal) || !getInput("robot_pose", robot_pose) ||
        !getInput("threshold", threshold_)) {
      return BT::NodeStatus::FAILURE;
    }
    const double dist =
        (goal.pose.translation() - robot_pose.pose.translation()).norm();
    return dist > threshold_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

 private:
  double threshold_;
};

/**
 * @brief 卡住检测条件 (spec §6.5): 低速 + 未到目标, 连续 `consecutive_stuck_ticks`
 * 次判定卡住 -> SUCCESS。
 *
 * 带记忆节点, 状态存 static (跨任务残留坑的根源)。每轮导航开始时由
 * `reset_stuck_token` 黑板令牌变化触发复位, 避免 static 状态跨任务泄漏。
 */
class IsRobotStuck : public BT::ConditionNode {
 public:
  IsRobotStuck(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config),
      stuck_velocity_threshold_(0.001),
      goal_tolerance_(0.5),
      consecutive_stuck_ticks_(50) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::Odometry>("robot_odometry", "robot odometry"),
      BT::InputPort<geometry_msgs::PoseStamped>("robot_pose", "current robot pose"),
      BT::InputPort<geometry_msgs::PoseStamped>(kGoal, "navigation goal"),
      BT::InputPort<uint64_t>(kResetStuckToken, 0u,
                              "blackboard token; changing it resets stuck state"),
      BT::InputPort<double>("stuck_velocity_threshold", 0.001,
                            "linear speed below which the robot is stalled (m/s)"),
      BT::InputPort<double>("goal_tolerance", 0.5, "goal tolerance in meters"),
      BT::InputPort<int>("consecutive_stuck_ticks", 50,
                         "consecutive ticks before declaring the robot stuck"),
    };
  }

  BT::NodeStatus tick() override {
    // 每轮导航开始时由 reset_stuck_token 令牌复位 static 状态 (spec §6.5)
    uint64_t token = 0;
    if (getInput<uint64_t>(kResetStuckToken, token)) {
      if (token != s_last_token_) {
        s_last_token_ = token;
        s_consecutive_stuck_ = 0;
      }
    }

    nav_msgs::Odometry odom;
    geometry_msgs::PoseStamped robot_pose, goal;
    if (!getInput("robot_odometry", odom) || !getInput("robot_pose", robot_pose) ||
        !getInput(kGoal, goal) ||
        !getInput("stuck_velocity_threshold", stuck_velocity_threshold_) ||
        !getInput("goal_tolerance", goal_tolerance_) ||
        !getInput("consecutive_stuck_ticks", consecutive_stuck_ticks_)) {
      return BT::NodeStatus::FAILURE;
    }

    const double speed = odom.twist.linear.norm();
    const double dist_to_goal =
        (goal.pose.translation() - robot_pose.pose.translation()).norm();
    if (speed < stuck_velocity_threshold_ && dist_to_goal > goal_tolerance_) {
      ++s_consecutive_stuck_;
      if (s_consecutive_stuck_ >= consecutive_stuck_ticks_) {
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }
    s_consecutive_stuck_ = 0;
    return BT::NodeStatus::FAILURE;
  }

 private:
  double stuck_velocity_threshold_;
  double goal_tolerance_;
  int consecutive_stuck_ticks_;

  // 跨任务 static 状态 —— 用 reset_stuck_token 令牌复位 (spec §6.5)
  inline static uint64_t s_last_token_ = 0;
  inline static int s_consecutive_stuck_ = 0;
};

/**
 * @brief 目标更新条件: goal 端口在 tick 之间变化 -> SUCCESS, 否则 FAILURE。
 *
 * 首次 tick 只建立基线 (init), 不判定为更新。
 */
class GoalUpdated : public BT::ConditionNode {
 public:
  GoalUpdated(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), initialized_(false) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::PoseStamped>(kGoal, "navigation goal") };
  }

  BT::NodeStatus tick() override {
    geometry_msgs::PoseStamped goal;
    if (!getInput(kGoal, goal)) {
      return BT::NodeStatus::FAILURE;
    }
    if (!initialized_) {
      initialized_ = true;
      last_goal_ = goal;
      return BT::NodeStatus::FAILURE;
    }
    if (goalChanged(goal)) {
      last_goal_ = goal;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

 private:
  static bool goalChanged(const geometry_msgs::PoseStamped& lhs,
                          const geometry_msgs::PoseStamped& rhs) {
    if (lhs.header.seq != rhs.header.seq) {
      return true;
    }
    if (lhs.header.frame_id != rhs.header.frame_id) {
      return true;
    }
    return !lhs.pose.isApprox(rhs.pose);
  }

  bool goalChanged(const geometry_msgs::PoseStamped& goal) const {
    return goalChanged(goal, last_goal_);
  }

  bool initialized_;
  geometry_msgs::PoseStamped last_goal_;
};

} // namespace behavior_tree
} // namespace ayanami
