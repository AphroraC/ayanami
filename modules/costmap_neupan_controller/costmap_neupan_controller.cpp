#include "costmap_neupan_controller.hpp"

#include <cmath>
#include <filesystem>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/logging.hpp>

namespace ayanami {
namespace modules {

namespace {

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double yawOf(const geometry_msgs::PoseStamped& pose) {
  const auto& q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/// @brief Resolve a possibly-relative asset path against the deployment
/// root (the parent of the config directory), the same semantics the
/// costmap static_map loader uses.
std::string resolveAsset(const std::string& path) {
  if (path.empty() || path.front() == '/' ||
      path.find("://") != std::string::npos) {
    return path;
  }
  const std::string directory =
      GlobalConfig::instance()
          ? GlobalConfig::instance()->param<std::string>("global",
                                                         "config_path", ".")
          : ".";
  return (std::filesystem::path(directory).parent_path() / path).string();
}

}  // namespace

CostmapNeupanController::CostmapNeupanController() = default;

void CostmapNeupanController::buildPlanner() {
  neupan::NeuPANPlanner::Config config;
  config.kinematics = param.kinematics == "diff"
                          ? neupan::Kinematics::Diff
                          : neupan::Kinematics::Omni;
  config.receding = param.receding;
  config.step_time = param.step_time;
  config.ref_speed = param.ref_speed;
  config.collision_threshold = param.collision_threshold;
  config.max_speed = neupan::Vec2(param.max_speed_v, param.max_speed_phi);
  config.max_acce = neupan::Vec2(param.max_acce_v, param.max_acce_phi);
  config.length = param.length;
  config.width = param.width;
  config.ipath.interval = param.ipath_interval;
  config.ipath.arrive_threshold = effective_arrive_threshold;
  config.ipath.ind_range = param.ind_range;
  config.pan.iter_num = param.iter_num;
  config.pan.dune_max_num = param.dune_max_num;
  config.pan.iter_threshold = param.iter_threshold;
  config.adjust.max_num = param.nrmp_max_num;
  config.adjust.q_s = neupan::Vec3::Constant(param.q_s);
  config.adjust.p_u = param.p_u;
  config.adjust.eta = param.eta;
  config.adjust.d_min = param.d_min;
  config.adjust.d_max = param.d_max;
  config.adjust.ro_obs = param.ro_obs;
  config.adjust.bk = param.bk;
  config.adjust.solver.eps_abs = param.osqp_eps_abs;
  config.adjust.solver.eps_rel = param.osqp_eps_rel;
  config.adjust.solver.max_iter = param.osqp_max_iter;
  config.adjust.solver.time_limit = param.osqp_time_limit;
  config.adjust.solver.polish = param.osqp_polish;
  config.adjust.solver.verbose = param.osqp_verbose;
  config.dune_checkpoint = resolveAsset(param.dune_checkpoint);
  planner = std::make_unique<neupan::NeuPANPlanner>(config);
}

void CostmapNeupanController::configureParameters(
    const ayanami::ModuleContext& context) {
  const Config& cfg = context.config;
  const std::string& s = context.name;
  param.receding = cfg.param<int>(s, "receding", param.receding);
  param.step_time = cfg.param<double>(s, "step_time", param.step_time);
  param.ref_speed = cfg.param<double>(s, "ref_speed", param.ref_speed);
  param.collision_threshold =
      cfg.param<double>(s, "collision_threshold", param.collision_threshold);
  param.length = cfg.param<double>(s, "length", param.length);
  param.width = cfg.param<double>(s, "width", param.width);
  param.max_speed_v =
      cfg.param<double>(s, "max_speed_v", param.max_speed_v);
  param.max_speed_phi =
      cfg.param<double>(s, "max_speed_phi", param.max_speed_phi);
  param.max_acce_v = cfg.param<double>(s, "max_acce_v", param.max_acce_v);
  param.max_acce_phi =
      cfg.param<double>(s, "max_acce_phi", param.max_acce_phi);
  param.kinematics = cfg.param<std::string>(s, "kinematics", param.kinematics);
  param.dune_checkpoint =
      cfg.param<std::string>(s, "dune_checkpoint", param.dune_checkpoint);
  param.ipath_interval =
      cfg.param<double>(s, "ipath_interval", param.ipath_interval);
  param.arrive_threshold =
      cfg.param<double>(s, "arrive_threshold", param.arrive_threshold);
  param.ind_range = cfg.param<int>(s, "ind_range", param.ind_range);
  param.iter_num = cfg.param<int>(s, "iter_num", param.iter_num);
  param.dune_max_num = cfg.param<int>(s, "dune_max_num", param.dune_max_num);
  param.nrmp_max_num =
      cfg.param<int>(s, "nrmp_max_num", param.nrmp_max_num);
  param.iter_threshold =
      cfg.param<double>(s, "iter_threshold", param.iter_threshold);
  param.q_s = cfg.param<double>(s, "q_s", param.q_s);
  param.p_u = cfg.param<double>(s, "p_u", param.p_u);
  param.eta = cfg.param<double>(s, "eta", param.eta);
  param.d_min = cfg.param<double>(s, "d_min", param.d_min);
  param.d_max = cfg.param<double>(s, "d_max", param.d_max);
  param.ro_obs = cfg.param<double>(s, "ro_obs", param.ro_obs);
  param.bk = cfg.param<double>(s, "bk", param.bk);
  param.osqp_eps_abs =
      cfg.param<double>(s, "osqp_eps_abs", param.osqp_eps_abs);
  param.osqp_eps_rel =
      cfg.param<double>(s, "osqp_eps_rel", param.osqp_eps_rel);
  param.osqp_max_iter = cfg.param<int>(s, "osqp_max_iter", param.osqp_max_iter);
  param.osqp_time_limit =
      cfg.param<double>(s, "osqp_time_limit", param.osqp_time_limit);
  param.osqp_polish = cfg.param<bool>(s, "osqp_polish", param.osqp_polish);
  param.osqp_verbose = cfg.param<bool>(s, "osqp_verbose", param.osqp_verbose);
  param.feed_max_points =
      cfg.param<int>(s, "feed_max_points", param.feed_max_points);
  param.heading_kp = cfg.param<double>(s, "heading_kp", param.heading_kp);
  param.heading_deadband =
      cfg.param<double>(s, "heading_deadband", param.heading_deadband);
  param.max_angular_z =
      cfg.param<double>(s, "max_angular_z", param.max_angular_z);
  param.heading_lookahead =
      cfg.param<int>(s, "heading_lookahead", param.heading_lookahead);

  effective_arrive_threshold = param.arrive_threshold;
  if (injected_arrive_tolerance &&
      *injected_arrive_tolerance < effective_arrive_threshold) {
    effective_arrive_threshold = *injected_arrive_tolerance;
  }
  buildPlanner();
}

void CostmapNeupanController::configure(
    const ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  (void)context;
  map_context = std::move(map);
}

void CostmapNeupanController::collectObstaclePoints(
    const std::string& target_frame) {
  obstacle_points.resize(2, 0);
  if (!map_context) {
    return;
  }
  if (!map_context->snapshotLocalObstaclePresence(presence_snapshot)) {
    return;
  }

  // Count the nonzero cells first, then emit their world centers (frame =
  // the local map frame, typically odom) transformed into the pose frame.
  // One transform lookup outside any map lock; a missing transform drops
  // the feed for this cycle (nominal tracking only, warning throttled by
  // the controller execution's own diagnostics).
  Eigen::Isometry3d target_from_mask = Eigen::Isometry3d::Identity();
  if (!presence_snapshot.frame.empty() &&
      presence_snapshot.frame != target_frame) {
    if (!components::RobotStateController::instance().lookupTransform(
            target_frame, presence_snapshot.frame,
            ayanami::Timestamp::latest(), target_from_mask, 0.0)) {
      SPDLOG_LOGGER_WARN(
          ayanami::controllerLogger(),
          "CostmapNeupanController: no transform {} <- {}; obstacle feed "
          "skipped this cycle",
          target_frame, presence_snapshot.frame);
      return;
    }
  }

  const unsigned size_x = presence_snapshot.size_x;
  const unsigned size_y = presence_snapshot.size_y;
  const double res = presence_snapshot.resolution;
  const double ox = presence_snapshot.origin_x;
  const double oy = presence_snapshot.origin_y;
  const auto& cells = presence_snapshot.costs;

  int count = 0;
  for (unsigned i = 0; i < cells.size(); ++i) {
    if (cells[i] != 0) ++count;
  }
  if (count == 0) {
    return;
  }
  const int stride =
      std::max(1, (count + param.feed_max_points - 1) / param.feed_max_points);

  // First pass into a flat list, then pack the matrix (stride applied while
  // packing so the sampled subset is uniform over the nonzero set).
  const int cap = (count + stride - 1) / stride;
  obstacle_points.resize(2, static_cast<Eigen::Index>(cap));
  int emitted = 0;
  int seen = 0;
  for (unsigned my = 0; my < size_y; ++my) {
    for (unsigned mx = 0; mx < size_x; ++mx) {
      const unsigned char cell = cells[my * size_x + mx];
      if (cell == 0) {
        continue;
      }
      if (seen % stride != 0) {
        ++seen;
        continue;
      }
      ++seen;
      const Eigen::Vector3d world(ox + (mx + 0.5) * res,
                                  oy + (my + 0.5) * res, 0.0);
      const Eigen::Vector3d p = target_from_mask * world;
      obstacle_points(0, emitted) = p.x();
      obstacle_points(1, emitted) = p.y();
      ++emitted;
    }
  }
  obstacle_points.conservativeResize(2, emitted);
}

double CostmapNeupanController::headingRate(const neupan::Mat3X& ref_s,
                                            double x, double y,
                                            double yaw) const {
  if (ref_s.cols() == 0) {
    return 0.0;
  }
  const int col = std::min<int>(param.heading_lookahead,
                                static_cast<int>(ref_s.cols()) - 1);
  const double dx = ref_s(0, col) - x;
  const double dy = ref_s(1, col) - y;
  if (std::hypot(dx, dy) < param.heading_deadband) {
    return 0.0;  // reference on top of us: nothing to align to
  }
  const double err = normalizeAngle(std::atan2(dy, dx) - yaw);
  if (std::abs(err) < param.heading_deadband) {
    return 0.0;
  }
  const double rate = param.heading_kp * err;
  return std::clamp(rate, -param.max_angular_z, param.max_angular_z);
}

bool CostmapNeupanController::setPlan(const nav_msgs::Path& plan) {
  if (!planner || plan.poses.empty()) {
    return false;
  }
  std::vector<neupan::InitialPath::PathPoint> path;
  path.reserve(plan.poses.size());
  for (std::size_t i = 0; i < plan.poses.size(); ++i) {
    // Skip duplicate consecutive poses: a degenerate start == goal plan
    // (or repeated astar cells) would otherwise hand InitialPath fewer
    // than 2 distinct points and throw.
    if (i > 0 &&
        (plan.poses[i].pose.position - plan.poses[i - 1].pose.position)
                .norm() < 1e-6) {
      continue;
    }
    const auto& pose = plan.poses[i];
    double theta = yawOf(pose);
    // A unit quaternion carries no heading (astar output): use the local
    // tangent instead, falling back to the segment start at the ends.
    const double qw = pose.pose.orientation.w();
    const double qx = pose.pose.orientation.x();
    if (std::abs(qw - 1.0) < 1e-9 && std::abs(qx) < 1e-9) {
      const auto& prev =
          plan.poses[i > 0 ? i - 1 : i];
      const auto& next =
          plan.poses[i + 1 < plan.poses.size() ? i + 1 : i];
      theta = std::atan2(next.pose.position.y() - prev.pose.position.y(),
                         next.pose.position.x() - prev.pose.position.x());
    }
    neupan::InitialPath::PathPoint point;
    point << pose.pose.position.x(), pose.pose.position.y(), theta, 1.0;
    path.push_back(point);
  }
  if (path.size() < 2) {
    // Degenerate plan (start == goal within float noise): install
    // nothing and accept -- the goal checker runs before the controller
    // each cycle, so a genuinely-reached goal settles as ARRIVED_GOAL; a
    // checker disagreement surfaces as INVALID_INPUT from compute (the
    // patience/recovery path) instead of an exception.
    return true;
  }
  if (!planner->hasPath()) {
    planner->setInitialPath(std::move(path));
  } else {
    // In-place replan: keep progress + the warm-started nominal (matches
    // the controller execution's updatePlan semantics -- no cold restart).
    const neupan::Vec3 state = has_state
                                   ? last_state
                                   : neupan::Vec3(path.front()(0),
                                                  path.front()(1),
                                                  path.front()(2));
    planner->replaceInitialPath(std::move(path), state);
  }
  return true;
}

void CostmapNeupanController::setGoalTolerances(
    const ayanami::PoseTolerance& tolerance) {
  injected_arrive_tolerance = tolerance.position;
  if (!planner) {
    return;
  }
  if (tolerance_warned || !injected_arrive_tolerance ||
      *injected_arrive_tolerance >= param.arrive_threshold) {
    return;
  }
  // The planner's internal arrival would latch short of the checker's
  // acceptance region: rebuild once with the tighter threshold. Rebuilding
  // drops the previous attempt's path state, which the about-to-come
  // setPlan replaces anyway (the execution calls this before the run loop
  // consumes the new plan).
  SPDLOG_LOGGER_WARN(
      ayanami::controllerLogger(),
      "CostmapNeupanController: arrive_threshold {} exceeds the goal "
      "checker tolerance {}; rebuilding the planner with the tighter value",
      param.arrive_threshold, *injected_arrive_tolerance);
  tolerance_warned = true;
  effective_arrive_threshold = *injected_arrive_tolerance;
  buildPlanner();
}

void CostmapNeupanController::setCyclePeriod(double seconds) {
  if (period_warned || seconds <= 0.0 || !planner) {
    return;
  }
  const double ratio = seconds / planner->config().step_time;
  if (ratio > 3.0 || ratio < 1.0 / 3.0) {
    SPDLOG_LOGGER_WARN(
        ayanami::controllerLogger(),
        "CostmapNeupanController: control period {} s vs step_time {} s "
        "(ratio {:.1f}); the nominal warm start will be noticeably stale "
        "each cycle",
        seconds, planner->config().step_time, ratio);
    period_warned = true;
  }
}

bool CostmapNeupanController::cancel() {
  canceled.store(true);
  // The planner solves synchronously with an OSQP time_limit; nothing to
  // interrupt mid-call. Report cooperative support so the execution layer
  // knows cancellation is honored between cycles.
  return true;
}

ayanami::ResultT<void> CostmapNeupanController::computeVelocityCommand(
    const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::TwistStamped& velocity,
    geometry_msgs::TwistStamped& cmd) {
  (void)velocity;
  cmd = geometry_msgs::TwistStamped{};  // zero-before-publish discipline
  if (!planner) {
    return ayanami::ResultT<void>::failure(
        ayanami::ErrorCode::INTERNAL_ERROR,
        "CostmapNeupanController: planner is not configured");
  }
  if (!planner->hasPath()) {
    return ayanami::ResultT<void>::failure(
        ayanami::ErrorCode::INVALID_INPUT,
        "CostmapNeupanController: no plan");
  }
  canceled.store(false);

  last_state = neupan::Vec3(pose.pose.position.x(), pose.pose.position.y(),
                            yawOf(pose));
  has_state = true;

  const auto t0 = std::chrono::steady_clock::now();
  collectObstaclePoints(pose.header.frame_id);
  neupan::NeuPANPlanner::Info info;
  const neupan::Vec2 action = planner->forward(last_state, obstacle_points, info);

  cmd.header = pose.header;
  cmd.twist.linear.x() = action(0);
  cmd.twist.linear.y() = action(1);
  cmd.twist.angular.z() = headingRate(info.ref_s, last_state(0),
                                      last_state(1), last_state(2));

  if (info.arrive || info.stop) {
    // Zero the twist (NeuPAN already returned zeros): the goal checker owns
    // arrival; a stop means the footprint grazes an obstacle -- hold still
    // and let the checker / FSM patience decide. Both are NOT failures.
    cmd.twist.linear.setZero();
    cmd.twist.angular.z() = 0.0;
    return ayanami::ResultT<void>::success();
  }
  if (!info.solved) {
    // Degraded solve (nominal, no avoidance): report a failure code so the
    // controller execution's patience path supervises; the caller must not
    // publish this command.
    cmd.twist.linear.setZero();
    cmd.twist.angular.setZero();
    return ayanami::ResultT<void>::failure(
        ayanami::ErrorCode::NO_VALID_COMMAND,
        "CostmapNeupanController: PAN did not converge to a solved QP");
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_telemetry_at >= std::chrono::seconds(1)) {
    last_telemetry_at = now;
    const double solve_ms =
        std::chrono::duration<double, std::milli>(now - t0).count();
    SPDLOG_LOGGER_DEBUG(
        ayanami::controllerLogger(),
        "CostmapNeupanController: solve {:.2f} ms, {} points, min_distance "
        "{:.2f}",
        solve_ms, obstacle_points.cols(),
        std::isfinite(info.min_distance) ? info.min_distance : -1.0);
  }
  return ayanami::ResultT<void>::success();
}

}  // namespace modules
}  // namespace ayanami
