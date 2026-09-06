#pragma once

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/runtime_lookup.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <cstdint>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief 路径有效性条件。
 *
 * 有 NavigationRuntime 注入 (常驻树装配) 时走地图族真实校验
 * (NavigationRuntime::isPathValid → AbstractMapContext::isPathValid:
 * costmap = 界内且非 lethal/unknown); 无注入 (纯黑板单测) 退化为非空检查。
 */
class IsPathValid : public BT::ConditionNode {
 public:
  IsPathValid(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config),
      nav_server(lookupNavigationRuntime(config)) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<nav_msgs::Path>("path", "path to validate") };
  }

  BT::NodeStatus tick() override {
    nav_msgs::Path path;
    if (!getInput("path", path) || path.poses.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    if (nav_server && !nav_server->isPathValid(path)) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

 private:
  components::NavigationRuntime::Ptr nav_server;
};

/**
 * @brief 到达目标条件: 机器人位姿与 goal 距离 <= tolerance -> SUCCESS。
 */
class IsGoalReached : public BT::ConditionNode {
 public:
  IsGoalReached(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), tolerance(0.5) {}

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
        !getInput("tolerance", tolerance)) {
      return BT::NodeStatus::FAILURE;
    }
    const double dist =
        (goal.pose.position - robot_pose.pose.position).norm();
    return dist <= tolerance ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

 private:
  double tolerance;
};

/**
 * @brief 距目标仍远条件: 机器人位姿与 goal 距离 > threshold -> SUCCESS。
 */
class IsFarFromGoal : public BT::ConditionNode {
 public:
  IsFarFromGoal(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), threshold(0.5) {}

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
        !getInput("threshold", threshold)) {
      return BT::NodeStatus::FAILURE;
    }
    const double dist =
        (goal.pose.position - robot_pose.pose.position).norm();
    return dist > threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

 private:
  double threshold;
};

/**
 * @brief 卡住检测条件 : 低速 + 未到目标, 连续 `consecutive_stuck_ticks`
 * 次判定卡住 -> SUCCESS。
 *
 * 带记忆节点。状态为本节点实例成员 (非进程级 static): 树常驻单实例即
 * 导航会话状态; 每轮导航开始时由 `reset_stuck_token` 黑板令牌变化触发复位。
 */
class IsRobotStuck : public BT::ConditionNode {
 public:
  IsRobotStuck(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config),
      stuck_velocity_threshold(0.001),
      goal_tolerance(0.5),
      consecutive_stuck_ticks(50) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::Odometry>("robot_odometry", "robot odometry"),
      BT::InputPort<geometry_msgs::PoseStamped>("robot_pose", "current robot pose"),
      BT::InputPort<geometry_msgs::PoseStamped>(k_goal, "navigation goal"),
      BT::InputPort<uint64_t>(k_reset_stuck_token, 0u,
                              "blackboard token; changing it resets stuck state"),
      BT::InputPort<double>("stuck_velocity_threshold", 0.001,
                            "linear speed below which the robot is stalled (m/s)"),
      BT::InputPort<double>("goal_tolerance", 0.5, "goal tolerance in meters"),
      BT::InputPort<int>("consecutive_stuck_ticks", 50,
                         "consecutive ticks before declaring the robot stuck"),
    };
  }

  BT::NodeStatus tick() override {
    // 每轮导航开始时由 reset_stuck_token 令牌变化复位本实例状态。
    uint64_t token = 0;
    if (getInput<uint64_t>(k_reset_stuck_token, token)) {
      if (token != last_token) {
        last_token = token;
        consecutive_stuck = 0;
      }
    }

    nav_msgs::Odometry odom;
    geometry_msgs::PoseStamped robot_pose, goal;
    if (!getInput("robot_odometry", odom) || !getInput("robot_pose", robot_pose) ||
        !getInput(k_goal, goal) ||
        !getInput("stuck_velocity_threshold", stuck_velocity_threshold) ||
        !getInput("goal_tolerance", goal_tolerance) ||
        !getInput("consecutive_stuck_ticks", consecutive_stuck_ticks)) {
      return BT::NodeStatus::FAILURE;
    }

    const double speed = odom.twist.twist.linear.norm();
    const double dist_to_goal =
        (goal.pose.position - robot_pose.pose.position).norm();
    if (speed < stuck_velocity_threshold && dist_to_goal > goal_tolerance) {
      ++consecutive_stuck;
      if (consecutive_stuck >= consecutive_stuck_ticks) {
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }
    consecutive_stuck = 0;
    return BT::NodeStatus::FAILURE;
  }

 private:
  double stuck_velocity_threshold;
  double goal_tolerance;
  int consecutive_stuck_ticks;

  // 会话状态 (实例成员, 非 static): token 变化触发复位。
  uint64_t last_token{0};
  int consecutive_stuck{0};
};

/**
 * @brief 目标更新条件: goal 端口在 tick 之间变化 -> SUCCESS, 否则 FAILURE。
 *
 * 首次 tick 只建立基线 (init), 不判定为更新。
 */
class GoalUpdated : public BT::ConditionNode {
 public:
  GoalUpdated(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), initialized(false) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::PoseStamped>(k_goal, "navigation goal") };
  }

  BT::NodeStatus tick() override {
    geometry_msgs::PoseStamped goal;
    if (!getInput(k_goal, goal)) {
      return BT::NodeStatus::FAILURE;
    }
    if (!initialized) {
      initialized = true;
      last_goal = goal;
      return BT::NodeStatus::FAILURE;
    }
    if (goalChanged(goal)) {
      last_goal = goal;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

 private:
  static bool goalChanged(const geometry_msgs::PoseStamped& lhs,
                          const geometry_msgs::PoseStamped& rhs) {
    return !goalEquals(lhs, rhs);
  }

  bool goalChanged(const geometry_msgs::PoseStamped& goal) const {
    return goalChanged(goal, last_goal);
  }

  bool initialized;
  geometry_msgs::PoseStamped last_goal;
};

} // namespace behavior_tree
} // namespace ayanami
