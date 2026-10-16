#include "abstract_mpc_controller.hpp"

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/SparseCore>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ayanami {
namespace modules {

namespace {

double wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace

struct AbstractMpcController::Impl {
  // ---- arc-length parameterized plan ----
  struct Plan {
    std::vector<Eigen::Vector2d> pts;
    std::vector<double> s;       // cumulative arc length, s[0] = 0
    std::vector<double> speed;   // per-segment reference speed
    double total{0.0};

    bool empty() const { return pts.size() < 2; }
  } plan;

  // ---- reference / output buffers (rose omni_.Ctx) ----
  Eigen::MatrixXd xref;        // 2 x steps (positions)
  Eigen::MatrixXd dref;        // 2 x steps (velocity feedforward)
  Eigen::MatrixXd output;      // 2 x steps (solved velocities)
  Eigen::MatrixXd last_output;

  OsqpEigen::Solver solver;
  Eigen::VectorXd qp_gradient;
  Eigen::VectorXd qp_lower;
  Eigen::VectorXd qp_upper;
  Eigen::SparseMatrix<double> qp_hessian;
  int steps_cached{0};
  bool solver_initialized{false};

  Eigen::Vector2d posAt(double s) const {
    const auto& p = plan;
    if (p.empty()) return Eigen::Vector2d::Zero();
    if (s <= 0.0) return p.pts.front();
    if (s >= p.total) return p.pts.back();
    // upper_bound over cumulative arc length
    std::size_t i =
        static_cast<std::size_t>(
            std::upper_bound(p.s.begin(), p.s.end(), s) - p.s.begin()) - 1;
    const double seg = p.s[i + 1] - p.s[i];
    const double alpha = seg > 1e-9 ? (s - p.s[i]) / seg : 0.0;
    return p.pts[i] + alpha * (p.pts[i + 1] - p.pts[i]);
  }

  Eigen::Vector2d tangentAt(double s) const {
    const auto& p = plan;
    if (p.empty()) return Eigen::Vector2d::UnitX();
    if (s <= 0.0) {
      const Eigen::Vector2d t = p.pts[1] - p.pts[0];
      const double n = t.norm();
      if (n > 1e-9) return t / n;
      return Eigen::Vector2d(1.0, 0.0);
    }
    if (s >= p.total) {
      const Eigen::Vector2d t = p.pts.back() - p.pts[p.pts.size() - 2];
      const double n = t.norm();
      if (n > 1e-9) return t / n;
      return Eigen::Vector2d(1.0, 0.0);
    }
    std::size_t i =
        static_cast<std::size_t>(
            std::upper_bound(p.s.begin(), p.s.end(), s) - p.s.begin()) - 1;
    const Eigen::Vector2d t = p.pts[i + 1] - p.pts[i];
    const double n = t.norm();
    if (n > 1e-9) return t / n;
    return Eigen::Vector2d(1.0, 0.0);
  }

  std::size_t nearestIndex(const Eigen::Vector2d& robot) const {
    std::size_t best = 0;
    double best_d = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < plan.pts.size(); ++i) {
      const double d = (plan.pts[i] - robot).squaredNorm();
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    return best;
  }

  bool solveQp(const Eigen::Vector2d& robot,
               const AbstractMpcControllerParam& p);
};

bool AbstractMpcController::Impl::solveQp(
    const Eigen::Vector2d& robot, const AbstractMpcControllerParam& p) {
  const int steps = p.predict_steps;
  const double dt = p.predict_dt;
  if (steps <= 1) {
    return false;
  }
  const int dimx = 2 * steps;
  const int nx = dimx + 2 * steps;

  if (steps_cached != steps) {
    // Reallocation needed (structure + buffers sized by the horizon).
    solver_initialized = false;
    steps_cached = steps;
    xref = Eigen::MatrixXd::Zero(2, steps);
    dref = Eigen::MatrixXd::Zero(2, steps);
    output = Eigen::MatrixXd::Zero(2, steps);
    last_output = output;
  }

  qp_gradient.setZero(nx);

  // ---- gradient: -2Q xref, -2R dref ----
  for (int i = 0; i < steps; ++i) {
    const int xi = 2 * i;
    qp_gradient[xi + 0] = -2.0 * p.q_pos * xref(0, i);
    qp_gradient[xi + 1] = -2.0 * p.q_pos * xref(1, i);
  }
  for (int i = 0; i < steps; ++i) {
    const int ui = dimx + 2 * i;
    qp_gradient[ui + 0] = -2.0 * p.r_vel * dref(0, i);
    qp_gradient[ui + 1] = -2.0 * p.r_vel * dref(1, i);
  }

  // ---- Hessian: Q on states (x0 fixed by the equality, unpenalized),
  // R + Rd on controls, -Rd on adjacent-control couplings ----
  std::vector<Eigen::Triplet<double>> h_trip;
  h_trip.reserve(static_cast<std::size_t>(nx) * 4);
  for (int i = 1; i < steps; ++i) {
    const int xi = 2 * i;
    h_trip.emplace_back(xi + 0, xi + 0, 2.0 * p.q_pos);
    h_trip.emplace_back(xi + 1, xi + 1, 2.0 * p.q_pos);
  }
  for (int i = 0; i < steps; ++i) {
    const int ui = dimx + 2 * i;
    const double w = (i == 0 || i == steps - 1) ? p.rd_smooth : 2.0 * p.rd_smooth;
    h_trip.emplace_back(ui, ui, 2.0 * (p.r_vel + w));
    h_trip.emplace_back(ui + 1, ui + 1, 2.0 * (p.r_vel + w));
  }
  for (int i = 1; i < steps; ++i) {
    const int ui = dimx + 2 * i;
    const int up = dimx + 2 * (i - 1);
    h_trip.emplace_back(ui, up, -2.0 * p.rd_smooth);
    h_trip.emplace_back(up, ui, -2.0 * p.rd_smooth);
    h_trip.emplace_back(ui + 1, up + 1, -2.0 * p.rd_smooth);
    h_trip.emplace_back(up + 1, ui + 1, -2.0 * p.rd_smooth);
  }
  qp_hessian.resize(nx, nx);
  qp_hessian.setFromTriplets(h_trip.begin(), h_trip.end());
  qp_hessian.makeCompressed();

  // ---- bounds: initial state | dynamics | speed box | dv box ----
  const int dyn_rows = 2 * steps;
  const int speed_rows = 2 * steps;
  const int smooth_rows = 2 * (steps - 1);
  const int nc = dyn_rows + speed_rows + smooth_rows;

  qp_lower.setConstant(nc, -1e10);
  qp_upper.setConstant(nc, 1e10);
  qp_lower.segment(0, 2) = robot;
  qp_upper.segment(0, 2) = robot;
  for (int i = 1; i < steps; ++i) {
    const int r = 2 * i;
    qp_lower[r] = qp_upper[r] = 0.0;
    qp_lower[r + 1] = qp_upper[r + 1] = 0.0;
  }
  for (int i = 0; i < steps; ++i) {
    const int r = dyn_rows + 2 * i;
    qp_lower[r] = qp_lower[r + 1] = -p.max_speed;
    qp_upper[r] = qp_upper[r + 1] = p.max_speed;
  }
  const double max_dv = p.max_accel * dt;
  for (int i = 1; i < steps; ++i) {
    const int r = dyn_rows + speed_rows + 2 * (i - 1);
    qp_lower[r] = qp_lower[r + 1] = -max_dv;
    qp_upper[r] = qp_upper[r + 1] = max_dv;
  }

  if (!solver_initialized) {
    Eigen::SparseMatrix<double> a(nc, nx);
    std::vector<Eigen::Triplet<double>> a_trip;
    a_trip.emplace_back(0, 0, 1.0);
    a_trip.emplace_back(1, 1, 1.0);
    for (int i = 1; i < steps; ++i) {
      const int r = 2 * i;
      const int last = 2 * (i - 1);
      const int ucol = dimx + 2 * (i - 1);
      a_trip.emplace_back(r, r, 1.0);
      a_trip.emplace_back(r, last, -1.0);
      a_trip.emplace_back(r, ucol, -dt);
      a_trip.emplace_back(r + 1, r + 1, 1.0);
      a_trip.emplace_back(r + 1, last + 1, -1.0);
      a_trip.emplace_back(r + 1, ucol + 1, -dt);
    }
    int offset = dyn_rows;
    for (int i = 0; i < steps; ++i) {
      const int r = offset + 2 * i;
      const int c = dimx + 2 * i;
      a_trip.emplace_back(r, c, 1.0);
      a_trip.emplace_back(r + 1, c + 1, 1.0);
    }
    offset += speed_rows;
    for (int i = 1; i < steps; ++i) {
      const int r = offset + 2 * (i - 1);
      const int u = dimx + 2 * i;
      const int up = dimx + 2 * (i - 1);
      a_trip.emplace_back(r, u, 1.0);
      a_trip.emplace_back(r, up, -1.0);
      a_trip.emplace_back(r + 1, u + 1, 1.0);
      a_trip.emplace_back(r + 1, up + 1, -1.0);
    }
    a.setFromTriplets(a_trip.begin(), a_trip.end());
    a.makeCompressed();

    solver.settings()->setWarmStart(true);
    solver.settings()->setAdaptiveRho(true);
    solver.settings()->setMaxIteration(2000);
    solver.settings()->setAbsoluteTolerance(1e-4);
    solver.settings()->setRelativeTolerance(1e-4);
    solver.settings()->setVerbosity(false);

    solver.data()->setNumberOfVariables(nx);
    solver.data()->setNumberOfConstraints(nc);
    solver.data()->setLinearConstraintsMatrix(a);
    solver.data()->setHessianMatrix(qp_hessian);
    solver.data()->setGradient(qp_gradient);
    solver.data()->setLowerBound(qp_lower);
    solver.data()->setUpperBound(qp_upper);
    if (!solver.initSolver()) {
      solver_initialized = false;  // retry the full init path next cycle
      return false;
    }
    solver_initialized = true;
  } else {
    solver.updateHessianMatrix(qp_hessian);
    solver.updateGradient(qp_gradient);
    solver.updateBounds(qp_lower, qp_upper);
  }

  solver.solveProblem();
  const Eigen::VectorXd sol = solver.getSolution();
  if (!sol.allFinite()) {
    return false;
  }
  for (int i = 0; i < steps; ++i) {
    const int ui = dimx + 2 * i;
    output(0, i) = sol[ui];
    output(1, i) = sol[ui + 1];
  }
  return output.allFinite();
}

AbstractMpcController::AbstractMpcController()
    : impl(std::make_unique<Impl>()) {}

AbstractMpcController::~AbstractMpcController() = default;

bool AbstractMpcController::cancel() {
  canceled.store(true);
  return true;
}

void AbstractMpcController::configureParameters(
    const ayanami::ModuleContext& context) {
  const ayanami::Config& cfg = context.config;
  const std::string& sec = context.name;
  param.predict_dt = cfg.param<double>(sec, "predict_dt", param.predict_dt);
  param.predict_steps =
      cfg.param<int>(sec, "predict_steps", param.predict_steps);
  param.max_iter = cfg.param<int>(sec, "max_iter", param.max_iter);
  param.max_speed = cfg.param<double>(sec, "max_speed", param.max_speed);
  param.max_accel = cfg.param<double>(sec, "max_accel", param.max_accel);
  param.delay_time = cfg.param<double>(sec, "delay_time", param.delay_time);
  param.blind_radius =
      cfg.param<double>(sec, "blind_radius", param.blind_radius);
  param.deviation_threshold = cfg.param<double>(sec, "deviation_threshold",
                                                param.deviation_threshold);
  param.cruise_speed =
      cfg.param<double>(sec, "cruise_speed", param.cruise_speed);
  param.q_pos = cfg.param<double>(sec, "q_pos", param.q_pos);
  param.r_vel = cfg.param<double>(sec, "r_vel", param.r_vel);
  param.rd_smooth = cfg.param<double>(sec, "rd_smooth", param.rd_smooth);
  param.angular_kp = cfg.param<double>(sec, "angular_kp", param.angular_kp);
  param.fixed_angular_z =
      cfg.param<double>(sec, "fixed_angular_z", param.fixed_angular_z);
  if (param.predict_dt <= 1e-3) param.predict_dt = 1e-3;
  if (param.predict_steps < 2) param.predict_steps = 2;
  if (param.max_iter < 1) param.max_iter = 1;
}

bool AbstractMpcController::setPlan(const nav_msgs::Path& plan_msg) {
  canceled.store(false);
  auto& p = impl->plan;
  p = Impl::Plan{};
  if (plan_msg.poses.size() < 2) {
    return false;
  }
  p.pts.reserve(plan_msg.poses.size());
  for (const auto& pose : plan_msg.poses) {
    p.pts.emplace_back(pose.pose.position.x(), pose.pose.position.y());
  }
  const std::size_t n = p.pts.size();
  p.s.assign(n, 0.0);
  p.speed.assign(n > 1 ? n - 1 : 1, param.cruise_speed);
  // Valid stamps (a smoother's time-parameterized output) drive the
  // per-segment reference speed; otherwise cruise_speed.
  const bool stamped =
      plan_msg.poses.front().header.stamp.count() > 0 &&
      plan_msg.poses.back().header.stamp.count() >
          plan_msg.poses.front().header.stamp.count();
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double ds = (p.pts[i + 1] - p.pts[i]).norm();
    p.s[i + 1] = p.s[i] + ds;
    if (stamped) {
      const double dt =
          static_cast<double>(plan_msg.poses[i + 1].header.stamp.count() -
                              plan_msg.poses[i].header.stamp.count()) *
          1e-9;
      if (dt > 1e-3 && dt < 1.0 && ds > 1e-6) {
        p.speed[i] =
            std::clamp(ds / dt, 0.0, std::max(param.max_speed, 0.1));
      }
    }
  }
  p.total = p.s.back();
  return p.total > 1e-6;
}

ResultT<void> AbstractMpcController::computeVelocityCommand(
    const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::TwistStamped& velocity,
    geometry_msgs::TwistStamped& cmd) {
  (void)velocity;  // pure tracker: no velocity feedback (rose parity)
  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED,
                                  "MPC controller canceled");
  }
  if (impl->plan.empty()) {
    return ResultT<void>::failure(ErrorCode::NO_VALID_COMMAND,
                                  "MPC controller has no plan");
  }
  const Eigen::Vector2d robot(pose.pose.position.x(), pose.pose.position.y());
  const Eigen::Matrix3d rot = pose.pose.orientation.toRotationMatrix();
  const double yaw = std::atan2(rot(1, 0), rot(0, 0));

  const std::size_t idx = impl->nearestIndex(robot);
  const double cross = (impl->plan.pts[idx] - robot).norm();
  if (cross > param.deviation_threshold) {
    // rose parity: the linear reference is unreliable this far off the
    // path -- hand the task back for a replan (NO_PROGRESS is recoverable).
    return ResultT<void>::failure(
        ErrorCode::NO_PROGRESS,
        "MPC: robot deviated " + std::to_string(cross) +
            " m from the path (threshold " +
            std::to_string(param.deviation_threshold) + " m)");
  }

  // ---- reference over the horizon (arc-length domain) ----
  const int steps = param.predict_steps;
  const double dt = param.predict_dt;
  const double ref_speed =
      std::clamp(impl->plan.speed[std::min(idx, impl->plan.speed.size() - 1)],
                 0.0, param.max_speed);
  const double s_start = impl->plan.s[idx] + ref_speed * param.delay_time;
  int k = 0;
  for (double s = s_start + ref_speed * dt;
       k < steps && s < impl->plan.total + 2.0 * param.blind_radius;
       s += ref_speed * dt) {
    const double sc = std::min(s, impl->plan.total);
    const Eigen::Vector2d pos = impl->posAt(sc);
    if ((pos - robot).norm() < param.blind_radius) {
      continue;  // rose: dedupe nearly coincident references
    }
    impl->xref.col(k) = pos;
    const Eigen::Vector2d tangent = impl->tangentAt(sc);
    impl->dref.col(k) = tangent * ref_speed;
    ++k;
  }
  for (; k < steps; ++k) {
    // Past the path end: hold the last point with zero reference velocity
    // (rose parity -- the controller decelerates onto the goal).
    impl->xref.col(k) = impl->posAt(impl->plan.total);
    impl->dref.col(k) = Eigen::Vector2d::Zero();
  }

  // ---- iterative solve: converge the control sequence (du early exit) ----
  for (int iter = 0; iter < param.max_iter; ++iter) {
    if (canceled.load()) {
      return ResultT<void>::failure(ErrorCode::CANCELED,
                                    "MPC controller canceled");
    }
    if (!impl->solveQp(robot, param)) {
      return ResultT<void>::failure(ErrorCode::INTERNAL_ERROR,
                                    "MPC QP solve failed");
    }
    if (iter > 0) {
      double du = 0.0;
      for (int c = 0; c < steps; ++c) {
        du += std::fabs(impl->output(0, c) - impl->last_output(0, c));
        du += std::fabs(impl->output(1, c) - impl->last_output(1, c));
      }
      if (du < 1e-4) {
        break;
      }
    }
    impl->last_output = impl->output;
  }

  const double vx_world = impl->output(0, 0);
  const double vy_world = impl->output(1, 0);
  cmd = geometry_msgs::TwistStamped{};
  cmd.header = pose.header;
  // world -> body
  cmd.twist.linear.x() = std::cos(yaw) * vx_world + std::sin(yaw) * vy_world;
  cmd.twist.linear.y() = -std::sin(yaw) * vx_world + std::cos(yaw) * vy_world;
  if (std::fabs(param.fixed_angular_z) > 1e-9) {
    cmd.twist.angular.z() = param.fixed_angular_z;
  } else {
    const Eigen::Vector2d tangent = impl->tangentAt(impl->plan.s[idx]);
    const double tangent_yaw = std::atan2(tangent.y(), tangent.x());
    cmd.twist.angular.z() = param.angular_kp * wrapAngle(tangent_yaw - yaw);
  }
  return ResultT<void>::success();
}

}  // namespace modules
}  // namespace ayanami
