// costmap_dwb_controller.cpp — 迁移自 dwb_core/src/dwb_local_planner.cpp
// (逐函数对应, ROS 组件全替换)。
#include "costmap_dwb_controller.hpp"

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include "conversions.hpp"
#include "exceptions.hpp"
#include "geometry_utils.hpp"
#include "registration.hpp"

using dwb_core::geometry_utils::euclidean_distance;

namespace ayanami {
namespace modules {

namespace {

/// @brief 等价 nav2_utils::transformPose 的 TransformStore 版:
/// 查 target ← source 变换 (latest), 返回目标帧下的 2D 位姿。
bool transformPose(ayanami::interfaces::TransformStore* store,
                   const std::string& target_frame,
                   const nav_2d_msgs::Pose2DStamped& pose_in,
                   nav_2d_msgs::Pose2DStamped& pose_out) {
  if (store == nullptr) {
    return false;
  }
  if (pose_in.frame_id == target_frame) {
    pose_out = pose_in;
    return true;
  }
  Eigen::Isometry3d T;
  if (!store->lookupTransform(target_frame, pose_in.frame_id,
                              ayanami::Timestamp::latest(), T)) {
    return false;
  }
  Eigen::Isometry3d T_in = Eigen::Isometry3d::Identity();
  T_in.translation() = Eigen::Vector3d(pose_in.pose.x, pose_in.pose.y, 0.0);
  T_in.linear() = Eigen::AngleAxisd(pose_in.pose.theta, Eigen::Vector3d::UnitZ())
                      .toRotationMatrix();
  const Eigen::Isometry3d T_out = T * T_in;
  pose_out.frame_id = target_frame;
  pose_out.pose.x = T_out.translation().x();
  pose_out.pose.y = T_out.translation().y();
  const Eigen::Quaterniond q(T_out.linear());
  pose_out.pose.theta =
      std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  return true;
}

}  // namespace

CostmapDwBLocalPlanner::CostmapDwBLocalPlanner() {
  registerAllCritics();
  registerAllTrajectoryGenerators();
}

void CostmapDwBLocalPlanner::initialize(
    const std::string& name, ayanami::costmap::CostmapContext::Ptr ctx) {
  ctx_ = std::move(ctx);
  dwb_plugin_name_ = name;

  // 模块配置 (经 ComponentConfig 当前插件装载上下文, 与 pid 模块同款)。
  const ayanami::ComponentConfig holder("config_controller");
  const ayanami::Config& cfg = holder.get();

  transform_tolerance_ =
      cfg.param<double>(dwb_plugin_name_, "transform_tolerance", 0.1);
  spdlog::info("Setting transform_tolerance to {}", transform_tolerance_);

  prune_plan_ = cfg.param<bool>(dwb_plugin_name_, "prune_plan", true);
  prune_distance_ = cfg.param<double>(dwb_plugin_name_, "prune_distance", 2.0);
  forward_prune_distance_ =
      cfg.param<double>(dwb_plugin_name_, "forward_prune_distance", 2.0);
  debug_trajectory_details_ =
      cfg.param<bool>(dwb_plugin_name_, "debug_trajectory_details", false);
  short_circuit_trajectory_evaluation_ = cfg.param<bool>(
      dwb_plugin_name_, "short_circuit_trajectory_evaluation", true);
  shorten_transformed_plan_ =
      cfg.param<bool>(dwb_plugin_name_, "shorten_transformed_plan", true);

  std::string traj_generator_name = cfg.param<std::string>(
      dwb_plugin_name_, "trajectory_generator_name",
      std::string("dwb_plugins::StandardTrajectoryGenerator"));

  costmap_ = ctx_ ? ctx_->localCostmap2D() : nullptr;
  if (costmap_ == nullptr) {
    throw std::runtime_error(
        "CostmapDwBLocalPlanner requires a local costmap (CostmapContext)");
  }

  // footprint: 从 local wrapper 的 LayeredCostmap 拿 (等价 costmap_ros_->
  // getRobotFootprint)。
  footprint_spec_.clear();
  if (ctx_ && ctx_->local_costmap_) {
    const auto& fp = ctx_->local_costmap_->get().getFootprint();
    for (const auto& point : fp) {
      footprint_spec_.push_back(::geometry_msgs::Point{point.x, point.y, 0.0});
    }
  }

  // 静态注册表创建 generator (strip "dwb_plugins::" 前缀)。
  const std::string kGenPrefix = "dwb_plugins::";
  std::string gen_key = traj_generator_name;
  if (gen_key.compare(0, kGenPrefix.size(), kGenPrefix) == 0) {
    gen_key = gen_key.substr(kGenPrefix.size());
  }
  auto& gen_registry = trajectoryGeneratorRegistry();
  const auto gen_it = gen_registry.find(gen_key);
  if (gen_it == gen_registry.end()) {
    throw std::runtime_error("Unknown trajectory generator: " +
                             traj_generator_name);
  }
  traj_generator_ = gen_it->second();
  traj_generator_->initialize(cfg, dwb_plugin_name_);

  try {
    loadCritics();
  } catch (const std::exception& e) {
    spdlog::error("Couldn't load critics! Caught exception: {}", e.what());
    throw;
  }
}

std::string CostmapDwBLocalPlanner::resolveCriticClassName(
    std::string base_name) {
  if (base_name.find("Critic") == std::string::npos) {
    base_name = base_name + "Critic";
  }

  if (base_name.find("::") == std::string::npos) {
    for (unsigned int j = 0; j < default_critic_namespaces_.size(); j++) {
      std::string full_name = default_critic_namespaces_[j] + "::" + base_name;
      // 注册表查简化名 (strip 命名空间)
      const std::string ns = default_critic_namespaces_[j] + "::";
      const std::string key = full_name.substr(ns.size());
      if (criticRegistry().count(key) != 0) {
        return full_name;
      }
    }
  }
  return base_name;
}

void CostmapDwBLocalPlanner::loadCritics() {
  // loadCritics 仅在 initialize 调用链内执行, 配置经局部 Config 读取
  // (与 onInit 的 config() 引用同生命周期)。
  const ayanami::ComponentConfig holder("config_controller");
  const ayanami::Config& cfg = holder.get();

  default_critic_namespaces_ =
      cfg.param<std::vector<std::string>>(
             dwb_plugin_name_, "default_critic_namespaces", {});
  if (default_critic_namespaces_.empty()) {
    default_critic_namespaces_.emplace_back("dwb_critics");
  }

  std::vector<std::string> critic_names =
      cfg.param<std::vector<std::string>>(dwb_plugin_name_, "critics", {});
  if (critic_names.empty()) {
    throw std::runtime_error("No critics defined for " + dwb_plugin_name_);
  }

  for (unsigned int i = 0; i < critic_names.size(); i++) {
    std::string critic_plugin_name = critic_names[i];
    std::string plugin_class = cfg.param<std::string>(
        dwb_plugin_name_, critic_plugin_name + ".class", critic_plugin_name);

    plugin_class = resolveCriticClassName(plugin_class);

    // 注册表键 = strip 命名空间后的简化类名
    const std::string kCriticPrefix = "dwb_critics::";
    std::string registry_key = plugin_class;
    if (registry_key.compare(0, kCriticPrefix.size(), kCriticPrefix) == 0) {
      registry_key = registry_key.substr(kCriticPrefix.size());
    }
    const auto it = criticRegistry().find(registry_key);
    if (it == criticRegistry().end()) {
      throw std::runtime_error("Unknown critic plugin class: " + plugin_class);
    }

    TrajectoryCritic::Ptr plugin = it->second();
    spdlog::info("Using critic \"{}\" ({})", critic_plugin_name, plugin_class);
    critics_.push_back(plugin);
    try {
      plugin->initialize(cfg, critic_plugin_name, dwb_plugin_name_, costmap_,
                         &footprint_spec_);
    } catch (const std::exception& e) {
      spdlog::error("Couldn't initialize critic plugin!");
      throw;
    }
    spdlog::info("Critic plugin initialized");
  }
}

bool CostmapDwBLocalPlanner::setPlan(const ayanami::nav_msgs::Path& path) {
  auto path2d = dwb_core::conversions::pathToPath2D(path);
  for (dwb_core::TrajectoryCritic::Ptr& critic : critics_) {
    critic->reset();
  }

  traj_generator_->reset();

  global_plan_ = path2d;
  return true;
}

FailureCode CostmapDwBLocalPlanner::computeVelocityCommands(
    const ayanami::geometry_msgs::PoseStamped& pose,
    const ayanami::geometry_msgs::TwistStamped& velocity,
    ayanami::geometry_msgs::TwistStamped& cmd_vel, std::string& message,
    const ayanami::abstract::AbstractGoalChecker::Ptr& /*goal_checker*/) {
  // 入口清零 (框架纪律): cmd_vel 全字段清零。
  cmd_vel = ayanami::geometry_msgs::TwistStamped{};

  std::shared_ptr<dwb_msgs::LocalPlanEvaluation> results = nullptr;

  try {
    nav_2d_msgs::Twist2DStamped cmd_vel2d = computeVelocityCommands(
        dwb_core::conversions::poseStampedToPose2D(pose),
        dwb_core::conversions::twist3Dto2D(velocity.twist), results);
    cmd_vel.header.frame_id = cmd_vel2d.frame_id;
    cmd_vel.twist = dwb_core::conversions::twist2Dto3D(cmd_vel2d.velocity);
    return FailureCode::None;
  } catch (const dwb_core::PlannerTFException& e) {
    message = e.what();
    return FailureCode::TransformUnavailable;
  } catch (const dwb_core::PlannerException& e) {
    message = e.what();
    return FailureCode::NoValidCommand;
  } catch (const std::exception& e) {
    message = e.what();
    return FailureCode::InternalError;
  }
}

nav_2d_msgs::Twist2DStamped CostmapDwBLocalPlanner::computeVelocityCommands(
    const nav_2d_msgs::Pose2DStamped& pose,
    const nav_2d_msgs::Twist2D& velocity,
    std::shared_ptr<dwb_msgs::LocalPlanEvaluation>& results) {
  if (results) {
    results->frame_id = pose.frame_id;
  }

  nav_2d_msgs::Path2D transformed_plan;
  nav_2d_msgs::Pose2DStamped goal_pose;

  prepareGlobalPlan(pose, transformed_plan, goal_pose);

  std::lock_guard<std::recursive_mutex> lock(
      ctx_->local_costmap_->mutex());

  for (dwb_core::TrajectoryCritic::Ptr& critic : critics_) {
    if (!critic->prepare(pose.pose, velocity, goal_pose.pose,
                         transformed_plan)) {
      spdlog::warn("A scoring function failed to prepare");
    }
  }

  try {
    dwb_msgs::TrajectoryScore best =
        coreScoringAlgorithm(pose.pose, velocity, results);

    // Return Value
    nav_2d_msgs::Twist2DStamped cmd_vel;
    cmd_vel.frame_id = pose.frame_id;
    cmd_vel.velocity = best.traj.velocity;

    // debrief stateful scoring functions
    for (dwb_core::TrajectoryCritic::Ptr& critic : critics_) {
      critic->debrief(cmd_vel.velocity);
    }

    return cmd_vel;
  } catch (const dwb_core::NoLegalTrajectoriesException& e) {
    nav_2d_msgs::Twist2D empty_cmd;
    // debrief stateful scoring functions
    for (dwb_core::TrajectoryCritic::Ptr& critic : critics_) {
      critic->debrief(empty_cmd);
    }

    throw;
  }
}

void CostmapDwBLocalPlanner::prepareGlobalPlan(
    const nav_2d_msgs::Pose2DStamped& pose,
    nav_2d_msgs::Path2D& transformed_plan,
    nav_2d_msgs::Pose2DStamped& goal_pose, bool /*publish_plan*/) {
  transformed_plan = transformGlobalPlan(pose);

  goal_pose.frame_id = global_plan_.frame_id;
  goal_pose.pose = global_plan_.poses.back();
  if (!transformPose(ctx_->transform_store_, ctx_->getGlobalFrame(),
                     goal_pose, goal_pose)) {
    throw dwb_core::PlannerTFException(
        "Unable to transform goal pose into local costmap's frame");
  }
}

dwb_msgs::TrajectoryScore CostmapDwBLocalPlanner::coreScoringAlgorithm(
    const ::geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D velocity,
    std::shared_ptr<dwb_msgs::LocalPlanEvaluation>& results) {
  nav_2d_msgs::Twist2D twist;
  dwb_msgs::Trajectory2D traj;
  dwb_msgs::TrajectoryScore best, worst;
  best.total = -1;
  worst.total = -1;
  dwb_core::IllegalTrajectoryTracker tracker;

  traj_generator_->startNewIteration(velocity);
  while (traj_generator_->hasMoreTwists()) {
    twist = traj_generator_->nextTwist();
    traj = traj_generator_->generateTrajectory(pose, velocity, twist);

    try {
      dwb_msgs::TrajectoryScore score = scoreTrajectory(traj, best.total);
      tracker.addLegalTrajectory();
      if (results) {
        results->twists.push_back(score);
      }
      if (best.total < 0 || score.total < best.total) {
        best = score;
        if (results) {
          results->best_index = results->twists.size() - 1;
        }
      }
      if (worst.total < 0 || score.total > worst.total) {
        worst = score;
        if (results) {
          results->worst_index = results->twists.size() - 1;
        }
      }
    } catch (const dwb_core::IllegalTrajectoryException& e) {
      if (results) {
        dwb_msgs::TrajectoryScore failed_score;
        failed_score.traj = traj;

        dwb_msgs::CriticScore cs;
        cs.name = e.getCriticName();
        cs.raw_score = -1.0;
        failed_score.scores.push_back(cs);
        failed_score.total = -1.0;
        results->twists.push_back(failed_score);
      }
      tracker.addIllegalTrajectory(e);
    }
  }

  if (best.total < 0) {
    if (debug_trajectory_details_) {
      spdlog::error("{}", tracker.getMessage());
      for (auto const& x : tracker.getPercentages()) {
        spdlog::error("{:.2f}: {:>10}/{}", x.second, x.first.first,
                      x.first.second);
      }
    }
    throw dwb_core::NoLegalTrajectoriesException(tracker);
  }

  return best;
}

dwb_msgs::TrajectoryScore CostmapDwBLocalPlanner::scoreTrajectory(
    const dwb_msgs::Trajectory2D& traj, double best_score) {
  dwb_msgs::TrajectoryScore score;
  score.traj = traj;

  for (dwb_core::TrajectoryCritic::Ptr& critic : critics_) {
    dwb_msgs::CriticScore cs;
    cs.name = critic->getName();
    cs.scale = critic->getScale();

    if (cs.scale == 0.0) {
      score.scores.push_back(cs);
      continue;
    }

    double critic_score = critic->scoreTrajectory(traj);
    cs.raw_score = critic_score;
    score.scores.push_back(cs);
    score.total += critic_score * cs.scale;
    if (short_circuit_trajectory_evaluation_ && best_score > 0 &&
        score.total > best_score) {
      // since we keep adding positives, once we are worse than the best, we
      // will stay worse
      break;
    }
  }

  return score;
}

nav_2d_msgs::Path2D CostmapDwBLocalPlanner::transformGlobalPlan(
    const nav_2d_msgs::Pose2DStamped& pose) {
  if (global_plan_.poses.empty()) {
    throw dwb_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  nav_2d_msgs::Pose2DStamped robot_pose;
  if (!transformPose(ctx_->transform_store_, global_plan_.frame_id, pose,
                     robot_pose)) {
    throw dwb_core::PlannerTFException(
        "Unable to transform robot pose into global plan's frame");
  }

  // we'll discard points on the plan that are outside the local costmap
  double dist_threshold =
      std::max(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()) *
      costmap_->getResolution() / 2.0;

  // If prune_plan is enabled (it is by default) then we want to restrict the
  // plan to distances within that range as well.
  double prune_dist = prune_distance_;

  // Set the maximum distance we'll include points before getting to the part
  // of the path where the robot is located (the start of the plan). Basically,
  // these are the points the robot has already passed.
  double transform_start_threshold;
  if (prune_plan_) {
    transform_start_threshold = std::min(dist_threshold, prune_dist);
  } else {
    transform_start_threshold = dist_threshold;
  }

  // Set the maximum distance we'll include points after the part of the plan
  // near the robot (the end of the plan). This determines the amount of the
  // plan passed on to the critics
  double transform_end_threshold;
  double forward_prune_dist = forward_prune_distance_;
  if (shorten_transformed_plan_) {
    transform_end_threshold = std::min(dist_threshold, forward_prune_dist);
  } else {
    transform_end_threshold = dist_threshold;
  }

  // Find the first pose in the global plan that's further than prune distance
  // from the robot using integrated distance
  auto prune_point = dwb_core::geometry_utils::first_after_integrated_distance(
      global_plan_.poses.begin(), global_plan_.poses.end(), prune_dist);

  // Find the first pose in the plan (upto prune_point) that's less than
  // transform_start_threshold from the robot.
  auto transformation_begin = std::find_if(
      begin(global_plan_.poses), prune_point,
      [&](const auto& global_plan_pose) {
        return euclidean_distance(robot_pose.pose, global_plan_pose) <
               transform_start_threshold;
      });

  // Find the first pose in the end of the plan that's further than
  // transform_end_threshold from the robot using integrated distance
  auto transformation_end = std::find_if(
      transformation_begin, global_plan_.poses.end(), [&](const auto& plan_pose) {
        return euclidean_distance(plan_pose, robot_pose.pose) >
               transform_end_threshold;
      });

  // Transform the near part of the global plan into the robot's frame of
  // reference.
  nav_2d_msgs::Path2D transformed_plan;
  transformed_plan.frame_id = ctx_->getGlobalFrame();
  transformed_plan.stamp = pose.stamp;

  // Helper function for the transform below. Converts a pose2D from global
  // frame to local
  auto transformGlobalPoseToLocal = [&](const auto& global_plan_pose) {
    nav_2d_msgs::Pose2DStamped stamped_pose, transformed_pose;
    stamped_pose.frame_id = global_plan_.frame_id;
    stamped_pose.pose = global_plan_pose;
    transformPose(ctx_->transform_store_, transformed_plan.frame_id,
                  stamped_pose, transformed_pose);
    return transformed_pose.pose;
  };

  std::transform(transformation_begin, transformation_end,
                 std::back_inserter(transformed_plan.poses),
                 transformGlobalPoseToLocal);

  // Remove the portion of the global plan that we've already passed so we
  // don't process it on the next iteration.
  if (prune_plan_) {
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  }

  if (transformed_plan.poses.empty()) {
    throw dwb_core::PlannerException("Resulting plan has 0 poses in it.");
  }
  return transformed_plan;
}

}  // namespace modules
}  // namespace ayanami
