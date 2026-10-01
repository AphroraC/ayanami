// modules/rosemap_mpc_controller -- omni-base LMPC tracking controller.
//
// Pure C++ core ported from rose_navigation control/lmpc.cpp (MIT):
// single-integrator omni model, dense-but-structured QP solved by
// OsqpEigen with warm start and a fixed-sparsity constraint matrix.
// The reference comes from the stamped path (family time-parameterization
// contract); the controller never queries the map.

#include "rosemap_mpc_controller.hpp"

#include <ayanami/utility/config.hpp>

// clang-format off
#include <OsqpEigen/OsqpEigen.h>
// clang-format on

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ayanami {
namespace modules {

namespace {
constexpr double k_no_bound = 1e10;  // OsqpEigen-friendly infinity surrogate
}  // namespace

RosemapMpcController::RosemapMpcController() = default;
RosemapMpcController::~RosemapMpcController() = default;

struct RosemapMpcController::QpState {
  OsqpEigen::Solver solver;
  Eigen::SparseMatrix<double> constraint_matrix;
  Eigen::SparseMatrix<double> hessian;
  Eigen::VectorXd gradient;
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;
  Eigen::MatrixXd xref{Eigen::MatrixXd::Zero(2, 1)};
  Eigen::MatrixXd dref{Eigen::MatrixXd::Zero(2, 1)};
  Eigen::MatrixXd output{Eigen::MatrixXd::Zero(2, 1)};
  Eigen::MatrixXd last_output{Eigen::MatrixXd::Zero(2, 1)};
  bool initialized{false};
};

void RosemapMpcController::configureParameters(
    const ayanami::ModuleContext& context) {
  const Config& file = context.config;
  const std::string& section = context.name;
  // predict_dt is the PREDICTION-horizon discretization (an algorithm
  // parameter like time_steps), NOT the execution cadence: the MPC does not
  // integrate its output over the control period, so it deliberately does
  // not consume AbstractController::setCyclePeriod.
  param.predict_dt =
      file.param<double>(section, "predict_dt", param.predict_dt);
  param.predict_steps =
      file.param<int>(section, "predict_steps", param.predict_steps);
  param.max_iter = file.param<int>(section, "max_iter", param.max_iter);
  param.max_speed = file.param<double>(section, "max_speed", param.max_speed);
  param.max_accel = file.param<double>(section, "max_accel", param.max_accel);
  param.delay_time =
      file.param<double>(section, "delay_time", param.delay_time);
  param.blind_radius =
      file.param<double>(section, "blind_radius", param.blind_radius);
  param.reference_deviation = file.param<double>(
      section, "reference_deviation", param.reference_deviation);
  param.q_position =
      file.param<double>(section, "q_position", param.q_position);
  param.r_control =
      file.param<double>(section, "r_control", param.r_control);
  param.rd_delta = file.param<double>(section, "rd_delta", param.rd_delta);
  param.angular_mode =
      file.param<std::string>(section, "angular_mode", param.angular_mode);
  param.angular_gain =
      file.param<double>(section, "angular_gain", param.angular_gain);
  param.goal_heading_radius = file.param<double>(section, "goal_heading_radius",
                                                 param.goal_heading_radius);
  param.fixed_angular_z = file.param<double>(section, "fixed_angular_z",
                                             param.fixed_angular_z);
}

void RosemapMpcController::configure(
    const ModuleContext& context,
    ayanami::rosemap::RosemapMapContext::ConstPtr map) {
  configureParameters(context);
  map_context = std::move(map);
}

bool RosemapMpcController::setPlan(const nav_msgs::Path& path) {
  if (path.poses.size() < 2) {
    return false;
  }
  // Densify the plan at resample_ds before projecting: upstream paths may
  // be SPARSE (a simplified A* polyline or a DegradeToRawPath passthrough),
  // and nearest-sample projection on a sparse polyline jumps discontinuously
  // (the robot stalls at the frozen tail until the next replan). Stamps are
  // interpolated proportionally, so the time-parameterization contract is
  // preserved.
  constexpr double k_resample_ds = 0.1;
  std::vector<Sample> raw;
  raw.reserve(path.poses.size());
  for (const auto& pose : path.poses) {
    Sample sample;
    sample.position = pose.pose.position.head<2>();
    sample.stamp = pose.header.stamp.toSeconds();
    raw.push_back(sample);
  }
  std::vector<Sample> samples;
  samples.reserve(raw.size() * 4);
  for (std::size_t i = 0; i + 1 < raw.size(); ++i) {
    const Sample& a = raw[i];
    const Sample& b = raw[i + 1];
    samples.push_back(a);
    const double span = b.stamp - a.stamp;
    const double length = (b.position - a.position).norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(length /
                                                                 k_resample_ds)));
    for (int s = 1; s < steps; ++s) {
      const double tau = static_cast<double>(s) /
                         static_cast<double>(steps);
      Sample mid;
      mid.position = a.position + tau * (b.position - a.position);
      mid.stamp = span > 1e-6 ? a.stamp + tau * span : a.stamp;
      samples.push_back(std::move(mid));
    }
    if (i + 2 == raw.size()) {
      samples.push_back(b);
    }
  }
  // Velocities by finite difference (positions move monotonically in time;
  // a non-monotone stamp run degrades to zero feedforward per segment).
  for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
    const double dt = samples[i + 1].stamp - samples[i].stamp;
    if (dt > 1e-6) {
      samples[i].velocity =
          (samples[i + 1].position - samples[i].position) / dt;
    }
  }
  if (samples.size() >= 2) {
    samples.back().velocity = samples[samples.size() - 2].velocity;
  }
  // The tail pose keeps the planner's goal orientation (rosemap_astar_planner
  // writes the goal yaw there): the endgame heading alignment spins the body
  // onto it once translation collapses near the tail.
  const Eigen::Matrix3d tail_rotation =
      path.poses.back().pose.orientation.toRotationMatrix();
  goal_yaw = std::atan2(tail_rotation(1, 0), tail_rotation(0, 0));
  reference = std::move(samples);
  reference_frame = path.header.frame_id;
  ++plan_id;
  // Reset the cached solution: the new reference must not inherit the old
  // control sequence (navi trajectory_id gating semantics).
  applied_plan_id = 0;
  if (qp) {
    qp->output.setZero(2, param.predict_steps + 1);
    qp->last_output = qp->output;
  }
  return true;
}

ResultT<void> RosemapMpcController::computeVelocityCommand(
    const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::TwistStamped& velocity,
    geometry_msgs::TwistStamped& cmd) {
  canceled.store(false);
  (void)velocity;
  if (!qp) {
    qp = std::make_unique<QpState>();
  }
  if (reference.size() < 2) {
    return ResultT<void>::failure(ErrorCode::NO_VALID_COMMAND,
                                  "no reference plan");
  }
  // Plan switch: the constraint sparsity is unchanged, so the solver
  // stays initialized (values update below; the warm start carries
  // continuity). Only the cached control sequence was reset in setPlan.
  applied_plan_id = plan_id;

  const double robot_yaw = [&pose]() {
    const Eigen::Matrix3d rotation = pose.pose.orientation.toRotationMatrix();
    return std::atan2(rotation(1, 0), rotation(0, 0));
  }();
  const Eigen::Vector2d robot_position = pose.pose.position.head<2>();

  // ---- project the robot onto the reference (nearest stamped sample) ----
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < reference.size(); ++i) {
    const double d = (reference[i].position - robot_position).norm();
    if (d < best) {
      best = d;
      nearest = i;
    }
  }
  if (best > param.reference_deviation) {
    return ResultT<void>::failure(
        ErrorCode::NO_VALID_COMMAND,
        "robot strayed beyond the reference deviation gate");
  }
  double t_current = reference[nearest].stamp;

  // ---- build the horizon reference (positions + velocity feedforward) ----
  const int steps = param.predict_steps;
  const double dt = param.predict_dt;
  if (qp->xref.cols() != steps + 1) {
    qp->xref = Eigen::MatrixXd::Zero(2, steps + 1);
    qp->dref = Eigen::MatrixXd::Zero(2, steps + 1);
    qp->output = Eigen::MatrixXd::Zero(2, steps + 1);
    qp->last_output = qp->output;
  }
  bool has_reference = false;
  // Past-tail steps freeze HERE, not on the robot: freezing on the robot
  // makes the endgame reference "stay where you are" and the robot stalls
  // short of the goal until the progress checker kills the task.
  Eigen::Vector2d tail_position = reference.back().position;
  double horizon_t = t_current + dt + param.delay_time;
  const double tail_stamp = reference.back().stamp;
  for (int i = 0; i < steps; ++i) {
    if (horizon_t <= tail_stamp) {
      // Linear interpolation between the bracketing samples.
      std::size_t upper = nearest + 1;
      while (upper + 1 < reference.size() &&
             reference[upper].stamp < horizon_t) {
        ++upper;
      }
      const Sample& a = reference[upper - 1];
      const Sample& b = reference[upper];
      const double span = std::max(1e-9, b.stamp - a.stamp);
      const double tau = std::clamp((horizon_t - a.stamp) / span, 0.0, 1.0);
      const Eigen::Vector2d position =
          a.position + tau * (b.position - a.position);
      const Eigen::Vector2d feedforward =
          a.velocity + tau * (b.velocity - a.velocity);
      if ((position - robot_position).norm() < param.blind_radius) {
        // Skip near-duplicate references (an all-duplicate horizon gives
        // the QP a degenerate tracking target).
        horizon_t += dt;
        --i;
        continue;
      }
      has_reference = true;
      qp->xref.col(i + 1) = position;
      qp->dref.col(i + 1) = feedforward;
      tail_position = position;
    } else {
      // Past the tail: freeze the last position at zero velocity so the
      // approach decelerates into the goal (arrival is the goal checker's
      // decision).
      qp->xref.col(i + 1) = tail_position;
      qp->dref.col(i + 1).setZero();
      has_reference = true;
    }
    horizon_t += dt;
  }
  // Column 0 tracks the current robot state.
  qp->xref.col(0) = robot_position;
  qp->dref.col(0).setZero();
  if (!has_reference) {
    return ResultT<void>::failure(ErrorCode::NO_VALID_COMMAND,
                                  "no reference inside the horizon");
  }

  // ---- QP assembly (rose lmpc solve_mpc_omni structure) ----
  const int dim_x = 2 * steps;
  const int dim_u = 2 * steps;
  const int n_vars = dim_x + dim_u;
  const int dyn_rows = 2 * steps;
  const int speed_rows = 2 * steps;
  const int smooth_rows = 2 * (steps - 1);
  const int n_constraints = dyn_rows + speed_rows + smooth_rows;

  qp->gradient.setZero(n_vars);
  for (int i = 0; i < steps; ++i) {
    const int xi = 2 * i;
    qp->gradient[xi + 0] = -2.0 * param.q_position * qp->xref(0, i);
    qp->gradient[xi + 1] = -2.0 * param.q_position * qp->xref(1, i);
  }
  for (int i = 0; i < steps; ++i) {
    const int ui = dim_x + 2 * i;
    qp->gradient[ui + 0] = -2.0 * param.r_control * qp->dref(0, i);
    qp->gradient[ui + 1] = -2.0 * param.r_control * qp->dref(1, i);
  }

  std::vector<Eigen::Triplet<double>> hessian_triplets;
  hessian_triplets.reserve(static_cast<std::size_t>(n_vars) * 4);
  // State tracking (column 0 = fixed initial state, no cost).
  for (int i = 0; i < steps; ++i) {
    if (i == 0) {
      continue;
    }
    const int xi = 2 * i;
    hessian_triplets.emplace_back(xi + 0, xi + 0, 2.0 * param.q_position);
    hessian_triplets.emplace_back(xi + 1, xi + 1, 2.0 * param.q_position);
  }
  // Control effort + speed smoothness (adjacent-control difference folded
  // into the diagonal R weights exactly like the reference).
  for (int i = 0; i < steps; ++i) {
    const int ui = dim_x + 2 * i;
    double w0 = param.r_control;
    double w1 = param.r_control;
    if (i == 0 || i == steps - 1) {
      w0 += param.rd_delta;
      w1 += param.rd_delta;
    } else {
      w0 += 2.0 * param.rd_delta;
      w1 += 2.0 * param.rd_delta;
    }
    hessian_triplets.emplace_back(ui, ui, 2.0 * w0);
    hessian_triplets.emplace_back(ui + 1, ui + 1, 2.0 * w1);
  }
  for (int i = 1; i < steps; ++i) {
    const int ui = dim_x + 2 * i;
    const int up = dim_x + 2 * (i - 1);
    hessian_triplets.emplace_back(ui, up, -2.0 * param.rd_delta);
    hessian_triplets.emplace_back(up, ui, -2.0 * param.rd_delta);
    hessian_triplets.emplace_back(ui + 1, up + 1, -2.0 * param.rd_delta);
    hessian_triplets.emplace_back(up + 1, ui + 1, -2.0 * param.rd_delta);
  }
  qp->hessian.resize(n_vars, n_vars);
  qp->hessian.setFromTriplets(hessian_triplets.begin(),
                              hessian_triplets.end());
  qp->hessian.makeCompressed();

  // Bounds.
  qp->lower_bound.setConstant(n_constraints, -k_no_bound);
  qp->upper_bound.setConstant(n_constraints, k_no_bound);
  // Initial state equality.
  qp->lower_bound.segment(0, 2) = robot_position;
  qp->upper_bound.segment(0, 2) = robot_position;
  // Dynamics equalities.
  for (int i = 1; i < steps; ++i) {
    const int r = 2 * i;
    qp->lower_bound[r] = 0.0;
    qp->upper_bound[r] = 0.0;
    qp->lower_bound[r + 1] = 0.0;
    qp->upper_bound[r + 1] = 0.0;
  }
  // Speed box.
  const int speed_offset = dyn_rows;
  for (int i = 0; i < steps; ++i) {
    const int r = speed_offset + 2 * i;
    qp->lower_bound[r + 0] = -param.max_speed;
    qp->upper_bound[r + 0] = param.max_speed;
    qp->lower_bound[r + 1] = -param.max_speed;
    qp->upper_bound[r + 1] = param.max_speed;
  }
  // Adjacent-control (acceleration) box.
  const int smooth_offset = speed_offset + speed_rows;
  const double max_dv = param.max_accel * dt;
  for (int i = 1; i < steps; ++i) {
    const int r = smooth_offset + 2 * (i - 1);
    qp->lower_bound[r + 0] = -max_dv;
    qp->upper_bound[r + 0] = max_dv;
    qp->lower_bound[r + 1] = -max_dv;
    qp->upper_bound[r + 1] = max_dv;
  }

  // Constraint matrix (fixed sparsity; built once per horizon size).
  if (!qp->initialized) {
    Eigen::SparseMatrix<double> a(n_constraints, n_vars);
    std::vector<Eigen::Triplet<double>> a_triplets;
    a_triplets.emplace_back(0, 0, 1.0);
    a_triplets.emplace_back(1, 1, 1.0);
    for (int i = 1; i < steps; ++i) {
      const int r = 2 * i;
      const int last = 2 * (i - 1);
      const int ucol = dim_x + 2 * (i - 1);
      a_triplets.emplace_back(r, r, 1.0);
      a_triplets.emplace_back(r, last, -1.0);
      a_triplets.emplace_back(r, ucol, -dt);
      a_triplets.emplace_back(r + 1, r + 1, 1.0);
      a_triplets.emplace_back(r + 1, last + 1, -1.0);
      a_triplets.emplace_back(r + 1, ucol + 1, -dt);
    }
    for (int i = 0; i < steps; ++i) {
      const int r = dyn_rows + 2 * i;
      const int c = dim_x + 2 * i;
      a_triplets.emplace_back(r, c, 1.0);
      a_triplets.emplace_back(r + 1, c + 1, 1.0);
    }
    for (int i = 1; i < steps; ++i) {
      const int r = smooth_offset + 2 * (i - 1);
      const int u = dim_x + 2 * i;
      const int up = dim_x + 2 * (i - 1);
      a_triplets.emplace_back(r, u, 1.0);
      a_triplets.emplace_back(r, up, -1.0);
      a_triplets.emplace_back(r + 1, u + 1, 1.0);
      a_triplets.emplace_back(r + 1, up + 1, -1.0);
    }
    a.setFromTriplets(a_triplets.begin(), a_triplets.end());
    a.makeCompressed();
    qp->constraint_matrix = a;

    qp->solver.settings()->setWarmStart(true);
    qp->solver.settings()->setAdaptiveRho(true);
    qp->solver.settings()->setMaxIteration(2000);
    qp->solver.settings()->setAbsoluteTolerance(1e-4);
    qp->solver.settings()->setRelativeTolerance(1e-4);
    qp->solver.settings()->setVerbosity(false);
    qp->solver.data()->setNumberOfVariables(n_vars);
    qp->solver.data()->setNumberOfConstraints(n_constraints);
    qp->solver.data()->setLinearConstraintsMatrix(qp->constraint_matrix);
    qp->solver.data()->setHessianMatrix(qp->hessian);
    qp->solver.data()->setGradient(qp->gradient);
    qp->solver.data()->setLowerBound(qp->lower_bound);
    qp->solver.data()->setUpperBound(qp->upper_bound);
    if (!qp->solver.initSolver()) {
      return ResultT<void>::failure(ErrorCode::INTERNAL_ERROR,
                                    "OSQP initialization failed");
    }
    qp->initialized = true;
  } else {
    qp->solver.updateHessianMatrix(qp->hessian);
    qp->solver.updateGradient(qp->gradient);
    qp->solver.updateBounds(qp->lower_bound, qp->upper_bound);
  }

  // ---- inner re-solve loop (bounded by max_iter) ----
  for (int iter = 0; iter < std::max(1, param.max_iter); ++iter) {
    if (iter > 0) {
      double control_delta = 0.0;
      for (int c = 0; c < qp->output.cols(); ++c) {
        control_delta += std::fabs(qp->output(0, c) - qp->last_output(0, c));
        control_delta += std::fabs(qp->output(1, c) - qp->last_output(1, c));
      }
      if (control_delta < 1e-4) {
        break;
      }
    }
    qp->last_output = qp->output;
    if (qp->solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
      return ResultT<void>::failure(ErrorCode::NO_VALID_COMMAND,
                                    "OSQP failed to solve");
    }
    const Eigen::VectorXd solution = qp->solver.getSolution();
    if (!solution.allFinite()) {
      return ResultT<void>::failure(ErrorCode::NO_VALID_COMMAND,
                                    "OSQP returned a non-finite solution");
    }
    for (int i = 0; i < steps; ++i) {
      const int ui = dim_x + 2 * i;
      qp->output(0, i) = solution[ui + 0];
      qp->output(1, i) = solution[ui + 1];
    }
  }

  // ---- output: first control (world) -> body frame + angular policy ----
  const Eigen::Vector2d world_velocity(qp->output(0, 0), qp->output(1, 0));
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  cmd.twist.linear.x() = cos_yaw * world_velocity.x() +
                         sin_yaw * world_velocity.y();
  cmd.twist.linear.y() = -sin_yaw * world_velocity.x() +
                         cos_yaw * world_velocity.y();
  if (param.angular_mode == "fixed") {
    cmd.twist.angular.z() = param.fixed_angular_z;
  } else if (param.angular_mode == "zero") {
    cmd.twist.angular.z() = 0.0;
  } else {
    // "heading": align the body x axis with the command velocity direction
    // (holonomic heading control). Once translation collapses near the path
    // tail, rotate onto the goal heading instead -- otherwise the goal
    // checker's angle tolerance can never close the task.
    if (world_velocity.norm() > 0.05) {
      const double target_yaw = std::atan2(world_velocity.y(),
                                           world_velocity.x());
      double error = target_yaw - robot_yaw;
      error = std::atan2(std::sin(error), std::cos(error));
      cmd.twist.angular.z() = param.angular_gain * error;
    } else if ((robot_position - reference.back().position).norm() <
               param.goal_heading_radius) {
      double error = goal_yaw - robot_yaw;
      error = std::atan2(std::sin(error), std::cos(error));
      cmd.twist.angular.z() = param.angular_gain * error;
    } else {
      cmd.twist.angular.z() = 0.0;
    }
  }
  return ResultT<void>::success();
}

}  // namespace modules
}  // namespace ayanami
