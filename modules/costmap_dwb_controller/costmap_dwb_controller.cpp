// costmap_dwb_controller.cpp — 迁移自 dwb_core/src/dwb_local_planner.cpp
// (逐函数对应, ROS 组件全替换)。
#include "costmap_dwb_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "conversions.hpp"
#include "exceptions.hpp"
#include "geometry_utils.hpp"
#include "registration.hpp"

using dwb_core::geometry_utils::euclideanDistance;

namespace ayanami {
namespace modules {

namespace {

/// @brief 等价 nav2_utils::transformPose 的 TransformStore 版:
/// 查 target ← source 变换 (latest), 返回目标帧下的 2D 位姿。
bool transformPose(ayanami::components::RobotStateController* state_controller,
                   const std::string& target_frame,
                   const nav_2d_msgs::Pose2DStamped& pose_in,
                   nav_2d_msgs::Pose2DStamped& pose_out) {
  if (state_controller == nullptr) {
    return false;
  }
  if (pose_in.frame_id == target_frame) {
    pose_out = pose_in;
    return true;
  }
  Eigen::Isometry3d T;
  if (!state_controller->lookupTransform(target_frame, pose_in.frame_id,
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

void CostmapDwBLocalPlanner::configure(
    const PluginContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  dwb_plugin_name = context.name;
  map_context = std::move(map);

  // 参数经 PluginContext 注入的自身配置读取 (插件不知道全局配置布局)。
  // json 段 = controller.name (插件实例名)。
  module_config = std::make_unique<Config>(std::move(context.config));
  const ayanami::Config& cfg = *module_config;
  const std::string section =
      cfg.param<std::string>("controller", "name", "costmap_dwb_controller");
  param.transform_tolerance =
      cfg.param<double>(section, "transform_tolerance", 0.1);
  param.prune_plan = cfg.param<bool>(section, "prune_plan", true);
  param.prune_distance =
      cfg.param<double>(section, "prune_distance", 2.0);
  param.forward_prune_distance =
      cfg.param<double>(section, "forward_prune_distance", 2.0);
  param.debug_trajectory_details =
      cfg.param<bool>(section, "debug_trajectory_details", false);
  param.short_circuit_trajectory_evaluation =
      cfg.param<bool>(section, "short_circuit_trajectory_evaluation", true);
  param.shorten_transformed_plan =
      cfg.param<bool>(section, "shorten_transformed_plan", true);
  param.trajectory_generator_name = cfg.param<std::string>(
      section, "trajectory_generator_name",
      std::string("dwb_plugins::StandardTrajectoryGenerator"));
  param.default_critic_namespaces =
      cfg.param<std::vector<std::string>>(section, "default_critic_namespaces",
                                          {});
  param.critics = cfg.param<std::vector<std::string>>(section, "critics", {});

  // 创建运行资源 (trajectory generator + critics)。子插件 (generator/critic)
  // 自身参数由其 initialize 读取。

  spdlog::info("Setting transform_tolerance to {}", param.transform_tolerance);

  // 静态注册表创建 generator (strip "dwb_plugins::" 前缀)。
  const std::string k_gen_prefix = "dwb_plugins::";
  std::string gen_key = param.trajectory_generator_name;
  if (gen_key.compare(0, k_gen_prefix.size(), k_gen_prefix) == 0) {
    gen_key = gen_key.substr(k_gen_prefix.size());
  }
  auto& gen_registry = trajectoryGeneratorRegistry();
  const auto gen_it = gen_registry.find(gen_key);
  if (gen_it == gen_registry.end()) {
    throw std::runtime_error("Unknown trajectory generator: " +
                             param.trajectory_generator_name);
  }
  traj_generator = gen_it->second();
  traj_generator->initialize(cfg, dwb_plugin_name);

  // footprint 每周期从快照刷新; 初始化阶段 critic 先以空网格/空 footprint
  // 装载, costmap/footprint 指针在每周期 prepare 前由控制器重绑。
  // local_grid 置空: 首个 computeVelocityCommands 会返回 MapUnavailable 直至
  // 拿到有效快照。
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

  // 与 loadCritics 同源回退: 配置未指定命名空间时默认 dwb_critics。
  std::vector<std::string> namespaces = param.default_critic_namespaces;
  if (namespaces.empty()) {
    namespaces.emplace_back("dwb_critics");
  }
  if (base_name.find("::") == std::string::npos) {
    for (unsigned int j = 0; j < namespaces.size(); j++) {
      std::string full_name = namespaces[j] + "::" + base_name;
      // 注册表查简化名 (strip 命名空间)
      const std::string prefix = namespaces[j] + "::";
      const std::string key = full_name.substr(prefix.size());
      if (criticRegistry().count(key) != 0) {
        return full_name;
      }
    }
  }
  return base_name;
}

void CostmapDwBLocalPlanner::loadCritics() {
  // loadCritics 在 initialize (configure) 调用链内创建 critic 运行资源;
  // 模块级参数 default_critic_namespaces / critics 已在构造函数读入 param。
  const ayanami::Config& cfg = *module_config;

  std::vector<std::string> default_critic_namespaces =
      param.default_critic_namespaces;
  if (default_critic_namespaces.empty()) {
    default_critic_namespaces.emplace_back("dwb_critics");
  }

  std::vector<std::string> critic_names = param.critics;
  if (critic_names.empty()) {
    throw std::runtime_error("No critics defined for " + dwb_plugin_name);
  }

  for (unsigned int i = 0; i < critic_names.size(); i++) {
    std::string critic_plugin_name = critic_names[i];
    std::string plugin_class = cfg.param<std::string>(
        dwb_plugin_name, critic_plugin_name + ".class", critic_plugin_name);

    plugin_class = resolveCriticClassName(plugin_class);

    // 注册表键 = strip 命名空间后的简化类名
    const std::string k_critic_prefix = "dwb_critics::";
    std::string registry_key = plugin_class;
    if (registry_key.compare(0, k_critic_prefix.size(), k_critic_prefix) == 0) {
      registry_key = registry_key.substr(k_critic_prefix.size());
    }
    const auto it = criticRegistry().find(registry_key);
    if (it == criticRegistry().end()) {
      throw std::runtime_error("Unknown critic plugin class: " + plugin_class);
    }

    TrajectoryCritic::Ptr plugin = it->second();
    spdlog::info("Using critic \"{}\" ({})", critic_plugin_name, plugin_class);
    critics.push_back(plugin);
    try {
      // costmap/footprint 此时为空: 每周期 prepare 前由控制器重绑
      // (critic::setCostmap + 更新 footprint_spec 内容), 初始化不再要求
      // 具体地图实例。
      plugin->initialize(cfg, critic_plugin_name, dwb_plugin_name, nullptr,
                         &footprint_spec);
    } catch (const std::exception& e) {
      spdlog::error("Couldn't initialize critic plugin!");
      throw;
    }
    spdlog::info("Critic plugin initialized");
  }
}

bool CostmapDwBLocalPlanner::setPlan(const ayanami::nav_msgs::Path& path) {
  auto path2d = dwb_core::conversions::pathToPath2D(path);
  for (dwb_core::TrajectoryCritic::Ptr& critic : critics) {
    critic->reset();
  }

  traj_generator->reset();

  global_plan = path2d;
  return true;
}

ControllerResultT CostmapDwBLocalPlanner::computeVelocityCommand(
    const ayanami::geometry_msgs::PoseStamped& pose,
    const ayanami::geometry_msgs::TwistStamped& velocity) {
  // 入口清零 (框架纪律)。
  ayanami::geometry_msgs::TwistStamped cmd_vel = {};

  // 锁纪律: 持 shared 锁的 local 读视图 (整个打分周期), 零拷贝; 地图写者
  // 等锁写入。视图必须为函数局部变量 —— 析构于本周期结束, 立即释放读锁;
  // 存成成员会把 shared 锁持有到插件销毁, 地图写者 (落图/滚动/清图) 永久
  // 阻塞 (实测卡死根因)。
  if (!map_context) {
    return ControllerResultT::failure(
        ErrorCode::MAP_UNAVAILABLE,
        "CostmapDwBLocalPlanner: no costmap module");
  }
  auto local_view = map_context->lockLocal();
  if (!local_view.valid()) {
    return ControllerResultT::failure(
        ErrorCode::MAP_UNAVAILABLE,
        "CostmapDwBLocalPlanner: no local costmap available");
  }
  local_grid = local_view.grid();
  local_frame = local_view.frame();

  // 每周期刷新 footprint (critics 持有 footprint_spec 地址, 内容就地更新)。
  footprint_spec.clear();
  const auto& fp = map_context->footprint();
  footprint_spec.reserve(fp.size());
  for (const auto& point : fp) {
    footprint_spec.push_back(
        ::geometry_msgs::Point{point.x, point.y, 0.0});
  }
  for (dwb_core::TrajectoryCritic::Ptr& critic : critics) {
    critic->setCostmap(local_grid);
  }

  std::shared_ptr<dwb_msgs::LocalPlanEvaluation> results = nullptr;

 // 调参可观测性 (对标 nav2 cost_cloud/trajectories): AYANAMI_DWB_DEBUG=1
 // 时构建逐轨迹逐 critic 评分, 并在成功路径上节流输出 best/worst 统计。
  static const bool debug_scoring = [] {
    const char* env = std::getenv("AYANAMI_DWB_DEBUG");
    return env != nullptr && env[0] != '\0' && env[0] != '0';
  }();
  if (debug_scoring) {
    results = std::make_shared<dwb_msgs::LocalPlanEvaluation>();
  }

  try {
    // The execution feeds the robot pose from the TF tree, which is in the
    // map frame. Trajectory simulation and critics must run in the local
    // costmap frame, or trajectories are scored against the wrong cells and
    // DWB settles into inefficient rotation-only commands.
    nav_2d_msgs::Pose2DStamped pose2d =
        dwb_core::conversions::poseStampedToPose2D(pose);
    if (pose2d.frame_id != local_frame) {
      nav_2d_msgs::Pose2DStamped pose_local;
      if (!transformPose(&ayanami::components::RobotStateController::instance(),
                         local_frame, pose2d, pose_local)) {
        return ControllerResultT::failure(
            ErrorCode::TRANSFORM_UNAVAILABLE,
            "CostmapDwBLocalPlanner: unable to transform robot pose into "
            "local costmap frame");
      }
      pose2d = pose_local;
    }
    nav_2d_msgs::Twist2DStamped cmd_vel2d = computeVelocityCommands(
        pose2d, dwb_core::conversions::twist3Dto2D(velocity.twist), results);
    cmd_vel.header.frame_id = cmd_vel2d.frame_id;
    cmd_vel.twist = dwb_core::conversions::twist2Dto3D(cmd_vel2d.velocity);
    if (results) {
 // 节流 1s: best/worst 评分 + 非法轨迹占比 (调参时定位"谁否掉了轨迹")
      static thread_local auto last_log = std::chrono::steady_clock::now() -
                                          std::chrono::seconds(1);
      const auto now = std::chrono::steady_clock::now();
      if (now - last_log >= std::chrono::seconds(1)) {
        last_log = now;
        spdlog::info(
            "DWB scoring: twists={} best_index={} best=({:.3f}) "
            "v=({:.2f},{:.2f},{:.2f})",
            results->twists.size(), results->best_index,
            results->twists.empty()
                ? -1.0
                : results->twists[results->best_index].total,
            cmd_vel.twist.linear.x(), cmd_vel.twist.linear.y(),
            cmd_vel.twist.angular.z());
      }
    }
    return ControllerResultT::success(ControllerCommand{cmd_vel});
  } catch (const dwb_core::PlannerTFException& e) {
    return ControllerResultT::failure(ErrorCode::TRANSFORM_UNAVAILABLE,
                                      e.what());
  } catch (const dwb_core::PlannerException& e) {
    return ControllerResultT::failure(ErrorCode::NO_VALID_COMMAND, e.what());
  } catch (const std::exception& e) {
    return ControllerResultT::failure(ErrorCode::INTERNAL_ERROR, e.what());
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

  for (dwb_core::TrajectoryCritic::Ptr& critic : critics) {
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
    for (dwb_core::TrajectoryCritic::Ptr& critic : critics) {
      critic->debrief(cmd_vel.velocity);
    }

    return cmd_vel;
  } catch (const dwb_core::NoLegalTrajectoriesException& e) {
    nav_2d_msgs::Twist2D empty_cmd;
    // debrief stateful scoring functions
    for (dwb_core::TrajectoryCritic::Ptr& critic : critics) {
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

  goal_pose.frame_id = global_plan.frame_id;
  goal_pose.pose = global_plan.poses.back();
  if (!transformPose(&ayanami::components::RobotStateController::instance(),
                     local_frame, goal_pose, goal_pose)) {
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

  traj_generator->startNewIteration(velocity);
  while (traj_generator->hasMoreTwists()) {
    twist = traj_generator->nextTwist();
    traj = traj_generator->generateTrajectory(pose, velocity, twist);

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
    if (param.debug_trajectory_details) {
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

  for (dwb_core::TrajectoryCritic::Ptr& critic : critics) {
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
    if (param.short_circuit_trajectory_evaluation && best_score > 0 &&
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
  if (global_plan.poses.empty()) {
    throw dwb_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  nav_2d_msgs::Pose2DStamped robot_pose;
  if (!transformPose(&ayanami::components::RobotStateController::instance(),
                     global_plan.frame_id, pose, robot_pose)) {
    throw dwb_core::PlannerTFException(
        "Unable to transform robot pose into global plan's frame");
  }

  // we'll discard points on the plan that are outside the local costmap
  double dist_threshold =
      std::max(local_grid->getSizeInCellsX(), local_grid->getSizeInCellsY()) *
      local_grid->getResolution() / 2.0;

  // If prune_plan is enabled (it is by default) then we want to restrict the
  // plan to distances within that range as well.
  double prune_dist = param.prune_distance;

  // Set the maximum distance we'll include points before getting to the part
  // of the path where the robot is located (the start of the plan). Basically,
  // these are the points the robot has already passed.
  double transform_start_threshold;
  if (param.prune_plan) {
    transform_start_threshold = std::min(dist_threshold, prune_dist);
  } else {
    transform_start_threshold = dist_threshold;
  }

  // Set the maximum distance we'll include points after the part of the plan
  // near the robot (the end of the plan). This determines the amount of the
  // plan passed on to the critics
  double transform_end_threshold;
  double forward_prune_dist = param.forward_prune_distance;
  if (param.shorten_transformed_plan) {
    transform_end_threshold = std::min(dist_threshold, forward_prune_dist);
  } else {
    transform_end_threshold = dist_threshold;
  }

  // Find the first pose in the global plan that's further than prune distance
  // from the robot using integrated distance
  auto prune_point = dwb_core::geometry_utils::first_after_integrated_distance(
      global_plan.poses.begin(), global_plan.poses.end(), prune_dist);

  // Find the first pose in the plan (upto prune_point) that's less than
  // transform_start_threshold from the robot.
  auto transformation_begin = std::find_if(
      begin(global_plan.poses), prune_point,
      [&](const auto& global_plan_pose) {
        return euclideanDistance(robot_pose.pose, global_plan_pose) <
               transform_start_threshold;
      });

  // Find the first pose in the end of the plan that's further than
  // transform_end_threshold from the robot using integrated distance
  auto transformation_end = std::find_if(
      transformation_begin, global_plan.poses.end(), [&](const auto& plan_pose) {
        return euclideanDistance(plan_pose, robot_pose.pose) >
               transform_end_threshold;
      });

  // Transform the near part of the global plan into the robot's frame of
  // reference.
  nav_2d_msgs::Path2D transformed_plan;
  transformed_plan.frame_id = local_frame;
  transformed_plan.stamp = pose.stamp;

  // Helper function for the transform below. Converts a pose2D from global
  // frame to local
  auto transformGlobalPoseToLocal = [&](const auto& global_plan_pose) {
    nav_2d_msgs::Pose2DStamped stamped_pose, transformed_pose;
    stamped_pose.frame_id = global_plan.frame_id;
    stamped_pose.pose = global_plan_pose;
    transformPose(&ayanami::components::RobotStateController::instance(),
                  transformed_plan.frame_id, stamped_pose, transformed_pose);
    return transformed_pose.pose;
  };

  std::transform(transformation_begin, transformation_end,
                 std::back_inserter(transformed_plan.poses),
                 transformGlobalPoseToLocal);

  // Remove the portion of the global plan that we've already passed so we
  // don't process it on the next iteration.
  if (param.prune_plan) {
    global_plan.poses.erase(begin(global_plan.poses), transformation_begin);
  }

  if (transformed_plan.poses.empty()) {
    throw dwb_core::PlannerException("Resulting plan has 0 poses in it.");
  }
  return transformed_plan;
}

}  // namespace modules
}  // namespace ayanami
