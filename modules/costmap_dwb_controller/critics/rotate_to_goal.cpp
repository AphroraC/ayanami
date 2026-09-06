// rotate_to_goal.cpp — 迁移自 dwb_critics/src/rotate_to_goal.cpp
// (angles::shortest_angular_distance 换数学实现)。
#include "rotate_to_goal.hpp"

#include <filesystem>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include "../exceptions.hpp"
#include "../trajectory_utils.hpp"
#include "ayanami/utility/config.hpp"

namespace dwb_critics {

inline double hypotSq(double dx, double dy) { return dx * dx + dy * dy; }

void RotateToGoalCritic::onInit() {
  xy_goal_tolerance =
      config().param<double>(dwb_plugin_name, "xy_goal_tolerance", 0.25);
  xy_goal_tolerance_sq = xy_goal_tolerance * xy_goal_tolerance;

 // 容差联动一致性校验: 本 critic 的切入半径应 <= goal_checker 的到达容差,
 // 否则接近终点时"原地转向"与"未判到达"的判定区间重叠 (nav2 靠参数手动
 // 对齐, 这里显式校验)。checker 配置不在本插件 PluginContext 内, 经
 // GlobalConfig 解析导航配置; 任何缺失都静默跳过 (不阻塞插件加载)。
  try {
    ayanami::Config nav_cfg(
        ayanami::GlobalConfig::getConfigPath("config_navigation"));
    const std::string checker_file = nav_cfg.param<std::string>(
        "navigation", "config_goal_checker", std::string());
    if (!checker_file.empty()) {
      const std::string dir = ayanami::GlobalConfig::instance()->param<
          std::string>("global", "config_path", std::string("."));
      const std::string checker_path =
          std::filesystem::path(checker_file).is_absolute()
              ? checker_file
              : dir + "/" + checker_file;
      ayanami::Config checker_cfg(checker_path);
      const std::string checker_name =
          checker_cfg.param<std::string>("goal_checker", "name",
                                         std::string());
      if (!checker_name.empty() &&
          checker_cfg.hasParam(checker_name, "position_tolerance")) {
        const double position_tolerance = checker_cfg.param<double>(
            checker_name, "position_tolerance", 0.0);
        if (xy_goal_tolerance > position_tolerance + 1e-9) {
          spdlog::warn(
              "RotateToGoal xy_goal_tolerance ({:.3f}) > goal_checker '{}'"
              " position_tolerance ({:.3f}): 接近终点时原地转向与到达判定"
              "区间重叠, 建议 xy_goal_tolerance <= position_tolerance",
              xy_goal_tolerance, checker_name, position_tolerance);
        }
      }
    }
  } catch (const std::exception& e) {
    spdlog::debug("RotateToGoal tolerance consistency check skipped: {}",
                  e.what());
  }
  double stopped_xy_velocity =
      config().param<double>(dwb_plugin_name, "trans_stopped_velocity", 0.25);
  stopped_xy_velocity_sq = stopped_xy_velocity * stopped_xy_velocity;
  slowing_factor =
      config().param<double>(dwb_plugin_name, name + ".slowing_factor", 5.0);
  lookahead_time =
      config().param<double>(dwb_plugin_name, name + ".lookahead_time", -1.0);
  reset();
}

void RotateToGoalCritic::reset() {
  in_window = false;
  rotating = false;
}

bool RotateToGoalCritic::prepare(const geometry_msgs::Pose2D& pose,
                                 const nav_2d_msgs::Twist2D& vel,
                                 const geometry_msgs::Pose2D& goal,
                                 const nav_2d_msgs::Path2D&) {
  double dxy_sq = hypotSq(pose.x - goal.x, pose.y - goal.y);
  in_window = in_window || dxy_sq <= xy_goal_tolerance_sq;
  current_xy_speed_sq = hypotSq(vel.x, vel.y);
  rotating = rotating ||
              (in_window && current_xy_speed_sq <= stopped_xy_velocity_sq);
  goal_yaw = goal.theta;
  return true;
}

double RotateToGoalCritic::scoreTrajectory(
    const dwb_msgs::Trajectory2D& traj) {
  // If we're not sufficiently close to the goal, we don't care what the twist
  // is
  if (!in_window) {
    return 0.0;
  } else if (!rotating) {
    double speed_sq = hypotSq(traj.velocity.x, traj.velocity.y);
    if (speed_sq >= current_xy_speed_sq) {
      throw dwb_core::IllegalTrajectoryException(name,
                                                 "Not slowing down near goal.");
    }
    return speed_sq * slowing_factor + scoreRotation(traj);
  }

  // If we're sufficiently close to the goal, any transforming velocity is
  // invalid
  if (fabs(traj.velocity.x) > 0 || fabs(traj.velocity.y) > 0) {
    throw dwb_core::IllegalTrajectoryException(name,
                                               "Nonrotation command near goal.");
  }

  return scoreRotation(traj);
}

double RotateToGoalCritic::scoreRotation(const dwb_msgs::Trajectory2D& traj) {
  if (traj.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(name, "Empty trajectory.");
  }

  double end_yaw;
  if (lookahead_time >= 0.0) {
    geometry_msgs::Pose2D eval_pose =
        dwb_core::projectPose(traj, lookahead_time);
    end_yaw = eval_pose.theta;
  } else {
    end_yaw = traj.poses.back().theta;
  }
  // angles::shortest_angular_distance(from, to): 归一化到 [-pi, pi] 的最短角差
  return fabs(std::remainder(end_yaw - goal_yaw, 2.0 * M_PI));
}

}  // namespace dwb_critics
