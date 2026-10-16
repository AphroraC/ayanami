// modules/costmap_mppi_controller -- MPPI local control in the 4D swerve
// control space with MPPI-H mode switching. See the header for the
// reference implementations, the mode-switching rule, the steer-state
// estimation discipline, and the map-access contract.

#include "costmap_mppi_controller.hpp"

#include <ayanami/utility/config.hpp>
#include <ayanami/utility/logging.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

namespace ayanami {
namespace modules {

namespace {
  namespace critics = ayanami::modules::mppi::critics;
constexpr double k_two_pi = 6.283185307179586476925286766559;
constexpr double k_nan = std::numeric_limits<double>::quiet_NaN();

double extractYaw(const Eigen::Quaterniond& orientation) {
  const Eigen::Vector3d forward = orientation * Eigen::Vector3d::UnitX();
  return std::atan2(forward.y(), forward.x());
}

double stdOverValues(const std::vector<double>& values) {
  if (values.size() < 2) {
    return 0.0;
  }
  const double mean =
      std::accumulate(values.begin(), values.end(), 0.0) /
      static_cast<double>(values.size());
  double squared = 0.0;
  for (const double value : values) {
    squared += (value - mean) * (value - mean);
  }
  return std::sqrt(squared / static_cast<double>(values.size() - 1));
}
}  // namespace

CostmapMppiController::CostmapMppiController() = default;

CostmapMppiController::~CostmapMppiController() = default;

void CostmapMppiController::configureParameters(
    const ayanami::ModuleContext& context) {
  // Parameter loading, separated from configuration: the loader calls this
  // right after a successful load, and every value lands in the module's
  // own CostmapMppiControllerParam member (whose field initializers are
  // the defaults for keys the config omits).
  const Config& file = context.config;
  const std::string& section = context.name;
  param.d_l = file.param<double>(section, "d_l", param.d_l);
  param.d_r = file.param<double>(section, "d_r", param.d_r);
  param.l_f = file.param<double>(section, "l_f", param.l_f);
  param.l_r = file.param<double>(section, "l_r", param.l_r);
  param.tire_radius =
      file.param<double>(section, "tire_radius", param.tire_radius);
  param.max_steer_rate = file.param<double>(section, "max_steer_rate",
                                            param.max_steer_rate);
  param.max_steer_angle = file.param<double>(section, "max_steer_angle",
                                             param.max_steer_angle);
  param.max_wheel_speed = file.param<double>(section, "max_wheel_speed",
                                             param.max_wheel_speed);
  param.batch = file.param<int>(section, "batch", param.batch);
  param.time_steps = file.param<int>(section, "time_steps", param.time_steps);
  param.dt = file.param<double>(section, "dt", param.dt);
  param.temperature =
      file.param<double>(section, "temperature", param.temperature);
  param.sigma_steer =
      file.param<double>(section, "sigma_steer", param.sigma_steer);
  param.sigma_wheel =
      file.param<double>(section, "sigma_wheel", param.sigma_wheel);
  param.adaptive_sigma = file.param<bool>(section, "adaptive_sigma",
                                          param.adaptive_sigma);
  param.max_vel_x =
      file.param<double>(section, "max_vel_x", param.max_vel_x);
  param.max_vel_y =
      file.param<double>(section, "max_vel_y", param.max_vel_y);
  param.max_vel_theta = file.param<double>(section, "max_vel_theta",
                                           param.max_vel_theta);
  param.h_mode = file.param<bool>(section, "h_mode", param.h_mode);
  param.h_dist_error =
      file.param<double>(section, "h_dist_error", param.h_dist_error);
  param.h_yaw_error =
      file.param<double>(section, "h_yaw_error", param.h_yaw_error);
  param.h_obstacle_radius = file.param<double>(section, "h_obstacle_radius",
                                               param.h_obstacle_radius);
  param.sigma_vx = file.param<double>(section, "sigma_vx", param.sigma_vx);
  param.sigma_vy = file.param<double>(section, "sigma_vy", param.sigma_vy);
  param.sigma_omega = file.param<double>(section, "sigma_omega",
                                         param.sigma_omega);
  param.angular_mode =
      file.param<std::string>(section, "angular_mode", param.angular_mode);
  param.fixed_angular_z = file.param<double>(section, "fixed_angular_z",
                                             param.fixed_angular_z);
  param.w_path_follow =
      file.param<double>(section, "w_path_follow", param.w_path_follow);
  param.w_goal = file.param<double>(section, "w_goal", param.w_goal);
  param.w_goal_angle =
      file.param<double>(section, "w_goal_angle", param.w_goal_angle);
  param.w_twirling =
      file.param<double>(section, "w_twirling", param.w_twirling);
  param.w_constraint =
      file.param<double>(section, "w_constraint", param.w_constraint);
  param.w_velocity_deadband = file.param<double>(
      section, "w_velocity_deadband", param.w_velocity_deadband);
  param.w_obstacles =
      file.param<double>(section, "w_obstacles", param.w_obstacles);
  param.goal_angle_radius = file.param<double>(section, "goal_angle_radius",
                                               param.goal_angle_radius);
  param.deadband_xy =
      file.param<double>(section, "deadband_xy", param.deadband_xy);
  param.deadband_theta = file.param<double>(section, "deadband_theta",
                                            param.deadband_theta);
  param.use_footprint =
      file.param<bool>(section, "use_footprint", param.use_footprint);
  param.path_spacing = file.param<double>(section, "path_spacing",
                                          param.path_spacing);
  param.cursor_jump_distance = file.param<double>(
      section, "cursor_jump_distance", param.cursor_jump_distance);
  param.goal_stop_radius = file.param<double>(section, "goal_stop_radius",
                                              param.goal_stop_radius);
  param.control_period = file.param<double>(section, "control_period",
                                            param.control_period);
  // Seed the runtime period from the config before any execution push.
  control_cycle = param.control_period;
  param.random_seed =
      file.param<int>(section, "random_seed", param.random_seed);

  // Bind the immutable model objects once the parameters are final.
  geometry_ = swerve::SwerveGeometry{param.d_l, param.d_r, param.l_f,
                                     param.l_r, param.tire_radius};
  limits_ = swerve::ControlLimits{param.max_steer_rate,
                                  param.max_steer_angle,
                                  param.max_wheel_speed};
  settings_.batch = std::max(1, param.batch);
  settings_.time_steps = std::max(2, param.time_steps);
  settings_.dt = std::max(1e-3, param.dt);
  settings_.temperature = std::max(1e-3, param.temperature);
  if (param.angular_mode == "fixed") {
    omega_override_ = param.fixed_angular_z;
  } else if (param.angular_mode == "zero") {
    omega_override_ = 0.0;
  } else {
    omega_override_ = k_nan;  // "free": omega comes from the sampled control
  }
  goal_stop_radius_ = std::max(0.0, param.goal_stop_radius);
  active_sigma_ = mppi::NoiseSigma{param.sigma_steer, param.sigma_wheel};
  active_sigma_3d_ = mppi::NoiseSigma3D{param.sigma_vx, param.sigma_vy,
                                        param.sigma_omega};
  sampler_ = mppi::Sampler(param.random_seed > 0
                               ? static_cast<unsigned int>(param.random_seed)
                               : std::random_device{}());
}

void CostmapMppiController::configure(
    const ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // Direct-construction callers (tests, or a host that builds the object
  // itself) never go through the loader's configureParameters call, so
  // bind the parameters here as well. Re-reading the same keys is
  // idempotent.
  configureParameters(context);
  map_context = std::move(map);
}

void CostmapMppiController::setGoalTolerances(
    const ayanami::PoseTolerance& tolerance) {
  if (!goal_tolerances_) {
    SPDLOG_LOGGER_INFO(
        ayanami::controllerLogger(),
        "CostmapMppiController: goal checker XY tolerance {:.3f} m bounds "
        "the translation stop radius (config goal_stop_radius {:.3f} m)",
        tolerance.position, param.goal_stop_radius);
  }
  goal_tolerances_ = tolerance;
  goal_stop_radius_ = std::min(goal_stop_radius_, tolerance.position);
}

void CostmapMppiController::setCyclePeriod(double seconds) {
  if (seconds > 0.0) {
    control_cycle = seconds;
  }
}

bool CostmapMppiController::setPlan(const nav_msgs::Path& plan) {
  if (plan.poses.empty()) {
    return false;
  }
  // Degenerate path (single pose / start == goal) is legal input: accept
  // it and install no reference -- the goal checker owns the arrival
  // verdict (nav2 accepts any non-empty path; the neupan controller made
  // the same fix). Rejecting here fails the whole follow branch and burns
  // the recovery budget on a goal the robot is already standing on.
  degenerate_path_ =
      plan.poses.size() < 2 ||
      (plan.poses.front().pose.position.head<2>() -
       plan.poses.back().pose.position.head<2>())
              .norm() < 1e-9;
  if (degenerate_path_) {
    // Keep the goal heading: the goal checker's angle tolerance still has
    // to be met, so the robot must be able to turn in place toward it.
    const auto& goal_pose = plan.poses.back().pose;
    degenerate_goal_yaw_ = std::atan2(
        2.0 * (goal_pose.orientation.w() * goal_pose.orientation.z() +
               goal_pose.orientation.x() * goal_pose.orientation.y()),
        1.0 - 2.0 * (goal_pose.orientation.y() * goal_pose.orientation.y() +
                     goal_pose.orientation.z() * goal_pose.orientation.z()));
    dense_path_.clear();
    path_cursor_ = 0;
    warm_start_4d_.clear();
    warm_start_3d_.clear();
    canceled_.store(false);
    return true;
  }
  densifyPath(plan);
  path_cursor_ = 0;
  warm_start_4d_.clear();
  warm_start_3d_.clear();
  canceled_.store(false);
  return true;
}

bool CostmapMppiController::cancel() {
  canceled_.store(true);
  return true;
}

void CostmapMppiController::densifyPath(const nav_msgs::Path& plan) {
  dense_path_.clear();
  dense_path_.reserve(plan.poses.size() * 4);
  auto push = [&](double x, double y) { dense_path_.emplace_back(x, y); };
  push(plan.poses.front().pose.position.x(),
       plan.poses.front().pose.position.y());
  for (std::size_t i = 1; i < plan.poses.size(); ++i) {
    const Eigen::Vector2d from(plan.poses[i - 1].pose.position.x(),
                               plan.poses[i - 1].pose.position.y());
    const Eigen::Vector2d to(plan.poses[i].pose.position.x(),
                             plan.poses[i].pose.position.y());
    const double length = (to - from).norm();
    const int pieces =
        std::max(1, static_cast<int>(std::ceil(length / param.path_spacing)));
    for (int s = 1; s <= pieces; ++s) {
      const Eigen::Vector2d p =
          from + (to - from) * (static_cast<double>(s) / pieces);
      push(p.x(), p.y());
    }
  }
  goal_xy_ = dense_path_.back();
  goal_yaw_ = extractYaw(plan.poses.back().pose.orientation);
}

std::size_t CostmapMppiController::advanceCursor(double x, double y) {
  if (dense_path_.empty()) {
    return 0;
  }
  // Monotone forward window; a distant robot (recovery displacement,
  // teleop) restarts the search from the path head.
  const double jump = param.cursor_jump_distance;
  if ((dense_path_[path_cursor_] - Eigen::Vector2d(x, y)).norm() > jump) {
    path_cursor_ = 0;
  }
  const std::size_t window = 30;
  const std::size_t end =
      std::min(dense_path_.size() - 1, path_cursor_ + window);
  double best = (dense_path_[path_cursor_] - Eigen::Vector2d(x, y)).norm();
  std::size_t best_index = path_cursor_;
  for (std::size_t probe = path_cursor_ + 1; probe <= end; ++probe) {
    const double distance =
        (dense_path_[probe] - Eigen::Vector2d(x, y)).norm();
    if (distance < best) {
      best = distance;
      best_index = probe;
    }
  }
  path_cursor_ = best_index;
  // Path tangent around the cursor (for the MPPI-H heading-error gate).
  const std::size_t behind = path_cursor_ > 2 ? path_cursor_ - 2 : 0;
  const std::size_t ahead =
      std::min(dense_path_.size() - 1, path_cursor_ + 2);
  const Eigen::Vector2d tangent = dense_path_[ahead] - dense_path_[behind];
  if (tangent.norm() > 1e-9) {
    path_tangent_ = tangent;
  }
  return best_index;
}

mppi::SolveMode CostmapMppiController::selectMode(
    double x, double y, double yaw,
    const std::function<unsigned char(double, double)>& cost_at) {
  if (!param.h_mode) {
    return mppi::SolveMode::Swerve4D;
  }
  const std::size_t cursor = std::min(path_cursor_, dense_path_.size() - 1);
  const double tracking_error =
      (dense_path_[cursor] - Eigen::Vector2d(x, y)).norm();
  const double path_yaw = std::atan2(path_tangent_.y(), path_tangent_.x());
  const double yaw_error = std::abs(std::remainder(path_yaw - yaw, k_two_pi));
  const bool near = obstacleNearby(x, y, cost_at);
  return mppi::selectSolveMode(tracking_error, yaw_error, near,
                               param.h_dist_error, param.h_yaw_error);
}

bool CostmapMppiController::obstacleNearby(
    double x, double y,
    const std::function<unsigned char(double, double)>& cost_at) const {
  if (!cost_at) {
    return false;
  }
  if (cost_at(x, y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return true;
  }
  for (int i = 0; i < 8; ++i) {
    const double angle = k_two_pi * static_cast<double>(i) / 8.0;
    if (cost_at(x + param.h_obstacle_radius * std::cos(angle),
                y + param.h_obstacle_radius * std::sin(angle)) >=
        costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return true;
    }
  }
  return false;
}

void CostmapMppiController::fillCriticData(mppi::CriticData& data) const {
  data.path = &dense_path_;
  data.path_cursor = path_cursor_;
  data.goal_xy = goal_xy_;
  data.goal_yaw = goal_yaw_;
  data.max_vel_x = param.max_vel_x;
  data.max_vel_y = param.max_vel_y;
  data.max_vel_theta = param.max_vel_theta;
  data.angular_free = std::isnan(omega_override_);
  data.goal_angle_radius = param.goal_angle_radius;
  data.deadband_xy = param.deadband_xy;
  data.deadband_theta = param.deadband_theta;
  data.w_path_follow = param.w_path_follow;
  data.w_goal = param.w_goal;
  data.w_goal_angle = param.w_goal_angle;
  data.w_twirling = param.w_twirling;
  data.w_constraint = param.w_constraint;
  data.w_velocity_deadband = param.w_velocity_deadband;
  data.w_obstacles = param.w_obstacles;
  // Bounding-square footprint (corners + center) in the body frame.
  if (param.use_footprint) {
    const double half_length = std::max(param.l_f, param.l_r);
    const double half_width = std::max(param.d_l, param.d_r);
    data.footprint_offsets = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(half_length, half_width),
        Eigen::Vector2d(half_length, -half_width),
        Eigen::Vector2d(-half_length, half_width),
        Eigen::Vector2d(-half_length, -half_width)};
  }
}

double CostmapMppiController::sequenceStd(
    const std::vector<double>& values) const {
  return stdOverValues(values);
}

void CostmapMppiController::adaptSigmas(
    const mppi::ControlSequence& sequence) {
  std::vector<double> column(sequence.size());
  auto gather = [&](int field) {
    for (std::size_t t = 0; t < sequence.size(); ++t) {
      const double* fields = reinterpret_cast<const double*>(&sequence[t]);
      column[t] = fields[field];
    }
  };
  gather(0);
  const double std_fl = sequenceStd(column);
  gather(1);
  const double std_rr = sequenceStd(column);
  gather(2);
  const double std_fl_vel = sequenceStd(column);
  gather(3);
  const double std_rr_vel = sequenceStd(column);
  const double steer_std = std::max(std_fl, std_rr);
  const double wheel_std = std::max(std_fl_vel, std_rr_vel);
  active_sigma_.steer = std::clamp(steer_std, 0.3 * param.sigma_steer,
                                   2.0 * param.sigma_steer);
  active_sigma_.wheel = std::clamp(wheel_std, 0.3 * param.sigma_wheel,
                                   2.0 * param.sigma_wheel);
}

void CostmapMppiController::adaptSigmas3D(
    const std::vector<mppi::Control3D>& sequence) {
  std::vector<double> column(sequence.size());
  auto gather = [&](int field) {
    for (std::size_t t = 0; t < sequence.size(); ++t) {
      const double* fields = reinterpret_cast<const double*>(&sequence[t]);
      column[t] = fields[field];
    }
  };
  gather(0);
  const double std_vx = sequenceStd(column);
  gather(1);
  const double std_vy = sequenceStd(column);
  gather(2);
  const double std_omega = sequenceStd(column);
  active_sigma_3d_.vx =
      std::clamp(std_vx, 0.3 * param.sigma_vx, 2.0 * param.sigma_vx);
  active_sigma_3d_.vy =
      std::clamp(std_vy, 0.3 * param.sigma_vy, 2.0 * param.sigma_vy);
  active_sigma_3d_.omega = std::clamp(std_omega, 0.3 * param.sigma_omega,
                                      2.0 * param.sigma_omega);
}

ResultT<void> CostmapMppiController::computeVelocityCommand(
    const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::TwistStamped& velocity,
    geometry_msgs::TwistStamped& cmd) {
  (void)velocity;
  // canceled_ persists until the next setPlan: the execution layer stops
  // calling after cancel(), and a fresh goal re-arms via setPlan.
  cmd = geometry_msgs::TwistStamped();  // all-zero; non-planar stays zero

  if (!map_context) {
    return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                  "CostmapMppiController has no costmap");
  }
  if (degenerate_path_) {
    // Start == goal position: zero translation, P-rotation toward the goal
    // heading (the goal checker's angle tolerance still has to close;
    // holding a full zero deadlocks it and the progress checker aborts).
    double e = degenerate_goal_yaw_ - std::atan2(
        2.0 * (pose.pose.orientation.w() * pose.pose.orientation.z() +
               pose.pose.orientation.x() * pose.pose.orientation.y()),
        1.0 - 2.0 * (pose.pose.orientation.y() * pose.pose.orientation.y() +
                     pose.pose.orientation.z() * pose.pose.orientation.z()));
    while (e > M_PI) e -= 2.0 * M_PI;
    while (e < -M_PI) e += 2.0 * M_PI;
    const double w_cap =
        param.max_vel_theta > 0.0 ? param.max_vel_theta : 1.0;
    cmd.twist.angular.z() =
        std::clamp(2.0 * e, -w_cap, w_cap);
    return ResultT<void>::success();
  }
  if (dense_path_.size() < 2) {
    return ResultT<void>::failure(ErrorCode::NO_VALID_COMMAND,
                                  "CostmapMppiController has no plan");
  }

  // Map read state: held global copy (generation-gated) + one short-lock
  // local snapshot per cycle; the batch below runs on the immutable pair.
  const std::uint64_t global_gen = map_context->globalGeneration();
  if (!have_global_copy_ || global_gen != global_copy_generation_) {
    if (map_context->snapshotGlobal(global_copy_)) {
      global_copy_generation_ = global_copy_.generation;
      have_global_copy_ = true;
    }
  }
  ayanami::costmap::FusedMapView view;
  view.global = &global_copy_;
  bool map_valid = have_global_copy_ && global_copy_.valid();
  if (map_context->snapshotLocal(local_snapshot_) && local_snapshot_.valid()) {
    if (map_context->localFromGlobal(view.local_from_global)) {
      view.local = &local_snapshot_;
      map_valid = true;
    }
  }
  view.prepare();
  // No usable grid: solve without obstacle costs (the BT-level IsPathValid
  // and the execution layer still cover the path).
  const std::function<unsigned char(double, double)> cost_at =
      map_valid
          ? std::function<unsigned char(double, double)>(
                [&view](double x, double y) { return view.costAt(x, y); })
          : std::function<unsigned char(double, double)>();

  if (canceled_.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED, "Control canceled");
  }

  const double x = pose.pose.position.x();
  const double y = pose.pose.position.y();
  const double yaw = extractYaw(pose.pose.orientation);
  advanceCursor(x, y);

  swerve::BodyTwist twist;
  if (!solve(pose, yaw, cost_at, obstacleNearby(x, y, cost_at), twist)) {
    return ResultT<void>::failure(ErrorCode::NO_VALID_COMMAND,
                                  "all MPPI samples rejected");
  }
  if (canceled_.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED, "Control canceled");
  }

  // Arrival: zero the translation inside the stop radius; omega keeps
  // coming from the model (goal-angle critic rotates the body onto the
  // goal heading; fixed-spin mode never stops).
  const double distance_to_goal = (goal_xy_ - Eigen::Vector2d(x, y)).norm();
  if (distance_to_goal <= goal_stop_radius_) {
    twist.vx = 0.0;
    twist.vy = 0.0;
  }

  // Output cleanup: deadband (swerve steer motors chatter on micro
  // commands), twist-space clamp (mirror of the execution-layer clamp),
  // non-planar components stay zero.
  auto deadband = [](double value, double threshold) {
    return std::abs(value) < threshold ? 0.0 : value;
  };
  twist.vx = deadband(twist.vx, param.deadband_xy);
  twist.vy = deadband(twist.vy, param.deadband_xy);
  twist.omega = deadband(twist.omega, param.deadband_theta);
  // A fully collapsed command restores the BASE sigmas: otherwise the
  // adaptation self-reinforces a stall (standstill -> converged sequence
  // -> shrunk sigma -> no exploration to escape with).
  if (twist.vx == 0.0 && twist.vy == 0.0 && twist.omega == 0.0) {
    active_sigma_ = mppi::NoiseSigma{param.sigma_steer, param.sigma_wheel};
    active_sigma_3d_ = mppi::NoiseSigma3D{param.sigma_vx, param.sigma_vy,
                                          param.sigma_omega};
  }
  twist.vx = std::clamp(twist.vx, -param.max_vel_x, param.max_vel_x);
  twist.vy = std::clamp(twist.vy, -param.max_vel_y, param.max_vel_y);
  twist.omega =
      std::clamp(twist.omega, -param.max_vel_theta, param.max_vel_theta);

  cmd.header = pose.header;
  cmd.twist.linear = Eigen::Vector3d(twist.vx, twist.vy, 0.0);
  cmd.twist.angular = Eigen::Vector3d(0.0, 0.0, twist.omega);

  // Wheel-level telemetry: the rigid-body inverse of the FINAL twist
  // (exactly what the chassis controller's own IK will reproduce).
  return ResultT<void>::success();
}

bool CostmapMppiController::solve(
    const geometry_msgs::PoseStamped& pose, double yaw,
    const std::function<unsigned char(double, double)>& cost_at,
    bool obstacle_nearby, swerve::BodyTwist& cmd_twist) {
  const double x = pose.pose.position.x();
  const double y = pose.pose.position.y();
  const mppi::State start{x, y, yaw};
  const int batch = settings_.batch;
  const int steps = settings_.time_steps;

  const mppi::SolveMode mode = selectMode(x, y, yaw, cost_at);
  if (mode != last_mode_) {
    // CONVERT the entering mode's stale sequence instead of clearing it:
    // a heuristic re-init aims straight at the goal and would cut corners
    // through obstacles exactly at obstacle-gated mode flips. The per-step
    // conversions are the models' own maps, so continuity is preserved.
    if (mode == mppi::SolveMode::Swerve4D) {
      // Entering 4D: the 3D sequence maps into the steer space (both
      // wheels steer along the twist direction at the twist speed).
      if (warm_start_4d_.size() !=
          static_cast<std::size_t>(settings_.time_steps)) {
        warm_start_4d_.clear();
      } else if (warm_start_3d_.size() ==
                 static_cast<std::size_t>(settings_.time_steps)) {
        for (std::size_t t = 0; t < warm_start_4d_.size(); ++t) {
          const double speed =
              std::hypot(warm_start_3d_[t].vx, warm_start_3d_[t].vy);
          const double steer =
              speed > 1e-6 ? std::atan2(warm_start_3d_[t].vy,
                                        warm_start_3d_[t].vx)
                           : warm_start_4d_[t].fl_steer;
          warm_start_4d_[t] = swerve::Control4D{steer, steer, speed, speed};
        }
      }
    } else {
      // Entering 3D: each 4D control converts to its executed twist.
      if (warm_start_3d_.size() !=
          static_cast<std::size_t>(settings_.time_steps)) {
        warm_start_3d_.clear();
      } else if (warm_start_4d_.size() ==
                 static_cast<std::size_t>(settings_.time_steps)) {
        for (std::size_t t = 0; t < warm_start_3d_.size(); ++t) {
          const swerve::BodyTwist twist = swerve::control4dToTwist(
              warm_start_4d_[t], geometry_);
          warm_start_3d_[t] = mppi::Control3D{twist.vx, twist.vy,
                                              twist.omega};
        }
      }
    }
    last_mode_ = mode;
  }

  mppi::CriticData data;
  fillCriticData(data);
  data.cost_at = cost_at;
  mppi::Trajectory trajectory;

  if (mode == mppi::SolveMode::Swerve4D) {
    // Warm start: shift the last optimum; a fresh plan starts from a
    // drive-toward-goal heuristic so the first cycle is not a standstill.
    if (warm_start_4d_.size() !=
        static_cast<std::size_t>(settings_.time_steps)) {
      warm_start_4d_.assign(static_cast<std::size_t>(settings_.time_steps),
                            swerve::Control4D{});
      // Aim at the path point ~one horizon ahead of the cursor (NOT the
      // goal): the init then follows the path shape through corners.
      const std::size_t cursor = std::min(path_cursor_,
                                          dense_path_.size() - 1);
      const double spacing = std::max(1e-3, param.path_spacing);
      const std::size_t lookahead = std::min(
          dense_path_.size() - 1,
          cursor + static_cast<std::size_t>(
                       settings_.time_steps * settings_.dt / spacing));
      const Eigen::Vector2d to_target =
          dense_path_[lookahead] - Eigen::Vector2d(x, y);
      const double distance = to_target.norm();
      const double body_angle =
          std::atan2(to_target.y(), to_target.x()) - yaw;
      const double speed = std::clamp(
          distance / (settings_.time_steps * settings_.dt), 0.0,
          std::min({param.max_vel_x, param.max_vel_y, param.max_wheel_speed}));
      for (auto& control : warm_start_4d_) {
        control.fl_steer = body_angle;
        control.rr_steer = body_angle;
        control.fl_vel = speed;
        control.rr_vel = speed;
      }
    } else {
      mppi::shiftSequence(warm_start_4d_);
    }

    // ---- MPPI batch (4D swerve space) ----
    mppi::NoiseSigma sigma = active_sigma_;
    if (!param.adaptive_sigma) {
      sigma = mppi::NoiseSigma{param.sigma_steer, param.sigma_wheel};
    }
    std::vector<double> noise;
    sampler_.fillNoise(noise, batch, steps, sigma);
    std::vector<mppi::ControlSequence> samples(
        static_cast<std::size_t>(batch));
    std::vector<double> costs(static_cast<std::size_t>(batch), 0.0);
    bool any_finite = false;
    double best_cost = std::numeric_limits<double>::infinity();
    std::size_t best_index = 0;
    for (int k = 0; k < batch; ++k) {
      if (canceled_.load()) {
        return false;
      }
      mppi::ControlSequence& sample = samples[static_cast<std::size_t>(k)];
      sample.resize(static_cast<std::size_t>(steps));
      for (int t = 0; t < steps; ++t) {
        const std::size_t base =
            (static_cast<std::size_t>(k) * steps + t) * 4;
        swerve::Control4D control = warm_start_4d_[static_cast<std::size_t>(t)];
        control.fl_steer += noise[base + 0];
        control.rr_steer += noise[base + 1];
        control.fl_vel += noise[base + 2];
        control.rr_vel += noise[base + 3];
        sample[static_cast<std::size_t>(t)] =
            swerve::clampControl(control, limits_);
      }
      mppi::rolloutTrajectory(start, steer_fl_, steer_rr_, sample, geometry_,
                              limits_, settings_.dt, omega_override_,
                              trajectory);
      const double cost = critics::totalCost(trajectory, data);
      costs[static_cast<std::size_t>(k)] = cost;
      if (std::isfinite(cost)) {
        any_finite = true;
        if (cost < best_cost) {
          best_cost = cost;
          best_index = static_cast<std::size_t>(k);
        }
      }
    }
    if (!any_finite) {
      // Every sample rejected (e.g. fully walled in): stand still this
      // cycle instead of emitting garbage; the recovery ladder owns the
      // escape.
      return false;
    }

    // Anchor: the zero-noise base rollout joins the weighted set, so the
    // update can never regress below the warm start when the noise is
    // harmful (the softmax then hands the base nearly all the weight).
    mppi::Trajectory base_trajectory;
    mppi::rolloutTrajectory(start, steer_fl_, steer_rr_, warm_start_4d_,
                            geometry_, limits_, settings_.dt,
                            omega_override_, base_trajectory);
    const double base_cost = critics::totalCost(base_trajectory, data);
    samples.push_back(warm_start_4d_);
    costs.push_back(std::isfinite(base_cost)
                        ? base_cost
                        : std::numeric_limits<double>::infinity());

    mppi::ControlSequence updated;
    if (!mppi::weightedUpdate(warm_start_4d_, samples, costs,
                              settings_.temperature, updated)) {
      updated = samples[best_index];  // fall back to the greedy sample
    }
    warm_start_4d_ = updated;
    if (param.adaptive_sigma) {
      adaptSigmas(updated);
    }

    // ---- Command from the first control of the updated sequence ----
    const double max_delta =
        param.max_steer_rate * std::max(1e-3, control_cycle);
    const double executed_fl = swerve::approachSteer(
        steer_fl_, updated.front().fl_steer, max_delta);
    const double executed_rr = swerve::approachSteer(
        steer_rr_, updated.front().rr_steer, max_delta);
    const swerve::Control4D executed{executed_fl, executed_rr,
                                     updated.front().fl_vel,
                                     updated.front().rr_vel};
    cmd_twist = swerve::control4dToTwist(executed, geometry_);
    if (!std::isnan(omega_override_)) {
      cmd_twist.omega = omega_override_;
    }
    // Steer-state estimate advances toward the commanded targets over the
    // control period (the chassis does the same between cycles).
    steer_fl_ = executed_fl;
    steer_rr_ = executed_rr;
    return true;
  }

  // ---- MPPI batch (3D twist space, MPPI-H) ----
  if (warm_start_3d_.size() !=
      static_cast<std::size_t>(settings_.time_steps)) {
    warm_start_3d_.assign(static_cast<std::size_t>(settings_.time_steps),
                          mppi::Control3D{});
    const std::size_t cursor = std::min(path_cursor_,
                                        dense_path_.size() - 1);
    const double spacing = std::max(1e-3, param.path_spacing);
    const std::size_t lookahead = std::min(
        dense_path_.size() - 1,
        cursor + static_cast<std::size_t>(
                     settings_.time_steps * settings_.dt / spacing));
    const Eigen::Vector2d to_target =
        dense_path_[lookahead] - Eigen::Vector2d(x, y);
    const double distance = to_target.norm();
    const double body_angle =
        std::atan2(to_target.y(), to_target.x()) - yaw;
    const double speed =
        std::clamp(distance / (settings_.time_steps * settings_.dt), 0.0,
                   std::min(param.max_vel_x, param.max_vel_y));
    for (auto& control : warm_start_3d_) {
      control.vx = speed * std::cos(body_angle);
      control.vy = speed * std::sin(body_angle);
      control.omega = 0.0;
    }
  } else {
    mppi::shiftSequence3D(warm_start_3d_);
  }

  mppi::NoiseSigma3D sigma = active_sigma_3d_;
  if (!param.adaptive_sigma) {
    sigma = mppi::NoiseSigma3D{param.sigma_vx, param.sigma_vy,
                               param.sigma_omega};
  }
  std::vector<double> noise;
  sampler_.fillNoise3D(noise, batch, steps, sigma);
  std::vector<std::vector<mppi::Control3D>> samples3(
      static_cast<std::size_t>(batch));
  std::vector<double> costs(static_cast<std::size_t>(batch), 0.0);
  bool any_finite = false;
  double best_cost = std::numeric_limits<double>::infinity();
  std::size_t best_index = 0;
  for (int k = 0; k < batch; ++k) {
    if (canceled_.load()) {
      return false;
    }
    auto& sample = samples3[static_cast<std::size_t>(k)];
    sample.resize(static_cast<std::size_t>(steps));
    for (int t = 0; t < steps; ++t) {
      const std::size_t base = (static_cast<std::size_t>(k) * steps + t) * 3;
      mppi::Control3D control = warm_start_3d_[static_cast<std::size_t>(t)];
      control.vx += noise[base + 0];
      control.vy += noise[base + 1];
      control.omega += noise[base + 2];
      sample[static_cast<std::size_t>(t)] = mppi::clampControl3D(
          control, param.max_vel_x, param.max_vel_y, param.max_vel_theta);
    }
    mppi::rolloutTrajectory3D(start, sample, settings_.dt, omega_override_,
                              trajectory);
    const double cost = critics::totalCost(trajectory, data);
    costs[static_cast<std::size_t>(k)] = cost;
    if (std::isfinite(cost)) {
      any_finite = true;
      if (cost < best_cost) {
        best_cost = cost;
        best_index = static_cast<std::size_t>(k);
      }
    }
  }
  if (!any_finite) {
    return false;
  }
  mppi::Trajectory base_trajectory3d;
  mppi::rolloutTrajectory3D(start, warm_start_3d_, settings_.dt,
                            omega_override_, base_trajectory3d);
  const double base_cost = critics::totalCost(base_trajectory3d, data);
  samples3.push_back(warm_start_3d_);
  costs.push_back(std::isfinite(base_cost)
                      ? base_cost
                      : std::numeric_limits<double>::infinity());

  std::vector<mppi::Control3D> updated3d;
  if (!mppi::weightedUpdate3D(warm_start_3d_, samples3, costs,
                              settings_.temperature, updated3d)) {
    updated3d = samples3[best_index];
  }
  warm_start_3d_ = updated3d;
  if (param.adaptive_sigma) {
    adaptSigmas3D(updated3d);
  }

  // The commanded twist IS the first 3D control (clamped); the steer-state
  // estimate tracks the wheel angles the chassis IK will imply.
  mppi::Control3D first =
      mppi::clampControl3D(updated3d.front(), param.max_vel_x,
                           param.max_vel_y, param.max_vel_theta);
  cmd_twist = swerve::BodyTwist{first.vx, first.vy, first.omega};
  if (!std::isnan(omega_override_)) {
    cmd_twist.omega = omega_override_;
  }
  const auto wheels = swerve::twistToWheels8D(cmd_twist, geometry_);
  const double max_delta =
      param.max_steer_rate * std::max(1e-3, control_cycle);
  steer_fl_ = swerve::approachSteer(steer_fl_, wheels.steer[0], max_delta);
  steer_rr_ = swerve::approachSteer(steer_rr_, wheels.steer[3], max_delta);
  return true;
}

}  // namespace modules
}  // namespace ayanami
