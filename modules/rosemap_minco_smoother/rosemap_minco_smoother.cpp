// modules/rosemap_minco_smoother -- MINCO minimum-snap smoother.
//
// Pure C++ (zero ROS). Cost/LBFGS wiring ported from rose_navigation
// traj_opt (MIT); hardening (freeze mask, wall-clock budget, hard
// validation with rollback) follows the navi_minco_bit minco_optimizer
// semantics. Map access = one immutable RosemapSnapshot held for the whole
// optimization; queries go through the family interpolation contract.

#include "rosemap_minco_smoother.hpp"

#include <ayanami/rosemap/core/rosemap_query.hpp>
#include <ayanami/rosemap/core/rosemap_snapshot.hpp>
#include <ayanami/utility/config.hpp>

#include "vendor/lbfgs.hpp"
#include "vendor/minco.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace ayanami {
namespace modules {

namespace {

// Smoothed-L1 hinge (f, df) for x > 0; returns false when inactive.
inline bool smoothedL1(const double x, const double mu, double& f, double& df) {
  if (x < 0.0) {
    return false;
  }
  if (x > mu) {
    f = x - 0.5 * mu;
    df = 1.0;
    return true;
  }
  const double xdmu = x / mu;
  const double sqrxdmu = xdmu * xdmu;
  const double mumxd2 = mu - 0.5 * x;
  f = mumxd2 * sqrxdmu * xdmu;
  df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
  return true;
}

inline void kahanSum(double& sum, double& compensation, const double value) {
  const double y = value - compensation;
  const double t = sum + y;
  compensation = (t - sum) - y;
  sum = t;
}

}  // namespace

void RosemapMincoSmoother::configureParameters(
    const ayanami::ModuleContext& context) {
  const Config& file = context.config;
  const std::string& section = context.name;
  param.safe_dist =
      file.param<double>(section, "safe_dist", param.safe_dist);
  param.weight_pos =
      file.param<double>(section, "weight_pos", param.weight_pos);
  param.weight_smooth =
      file.param<double>(section, "weight_smooth", param.weight_smooth);
  param.mu_l1 = file.param<double>(section, "mu_l1", param.mu_l1);
  param.freeze_window =
      file.param<double>(section, "freeze_window", param.freeze_window);
  param.output_dt =
      file.param<double>(section, "output_dt", param.output_dt);
  param.max_iterations =
      file.param<int>(section, "max_iterations", param.max_iterations);
  param.opt_accuracy =
      file.param<double>(section, "opt_accuracy", param.opt_accuracy);
  param.lbfgs_mem_size =
      file.param<int>(section, "lbfgs_mem_size", param.lbfgs_mem_size);
  param.severe_scale =
      file.param<double>(section, "severe_scale", param.severe_scale);
  param.validation_dt =
      file.param<double>(section, "validation_dt", param.validation_dt);
}

void RosemapMincoSmoother::configure(
    const ModuleContext& context,
    ayanami::rosemap::RosemapMapContext::ConstPtr map) {
  configureParameters(context);
  map_context = std::move(map);
}

bool RosemapMincoSmoother::cancel() {
  canceled.store(true);
  return true;
}

ResultT<void> RosemapMincoSmoother::smoothPath(nav_msgs::Path& path,
                                               double max_duration) {
  canceled.store(false);
  const double budget =
      (max_duration > 0.0 && std::isfinite(max_duration))
          ? max_duration
          : std::numeric_limits<double>::infinity();

  if (!map_context) {
    return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                  "RosemapMincoSmoother has no field module");
  }
  const auto snapshot = map_context->currentSnapshot();
  if (!snapshot || !snapshot->valid()) {
    return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                  "RosemapMincoSmoother has no field snapshot");
  }
  const int input_count = static_cast<int>(path.poses.size());
  if (input_count < 2) {
    return ResultT<void>::failure(ErrorCode::INVALID_INPUT,
                                  "path needs at least two poses");
  }
  if (input_count < 3) {
    // One segment: nothing to refine (the minco model needs interior
    // control points); pass the path through unchanged.
    auto out = ResultT<void>::success();
    out.message = "path too short to smooth; passed through unchanged";
    return out;
  }

  // Frame handling: transform the control points into the field frame once.
  Eigen::Isometry3d field_from_path = Eigen::Isometry3d::Identity();
  if (!map_context->fieldFromFrame(path.header.frame_id, field_from_path)) {
    return ResultT<void>::failure(
        ErrorCode::TRANSFORM_UNAVAILABLE,
        "no transform field('" + snapshot->frame + "') <- '" +
            path.header.frame_id + "'");
  }
  const int piece_num = input_count - 1;
  const int ctrl_num = piece_num - 1;

  // Points + times from the input path. Stamps are RELATIVE seconds from
  // path start (the family time-parameterization contract); a stamp run
  // that is not usable falls back to arc-length allocation at 1 m/s.
  Eigen::Matrix2Xd points(2, input_count);
  Eigen::VectorXd times(piece_num);
  bool times_usable = true;
  double previous_stamp =
      path.poses.front().header.stamp.toSeconds();
  for (int i = 0; i < input_count; ++i) {
    const Eigen::Vector3d world =
        field_from_path * path.poses[static_cast<std::size_t>(i)].pose.position;
    points(0, i) = world.x();
    points(1, i) = world.y();
    const double stamp = path.poses[static_cast<std::size_t>(i)].header.stamp.toSeconds();
    if (i > 0) {
      const double dt = stamp - previous_stamp;
      if (!std::isfinite(dt) || dt <= 1e-3) {
        times_usable = false;
      }
      times(i - 1) = dt;
      previous_stamp = stamp;
    }
  }
  if (!times_usable) {
    // Arc-length fallback at 1 m/s.
    for (int i = 0; i < piece_num; ++i) {
      times(i) = (points.col(i + 1) - points.col(i)).norm();
    }
  }

  // Freeze mask: control points inside freeze_window seconds of the start
  // keep their input position.
  std::vector<unsigned char> frozen(static_cast<std::size_t>(ctrl_num), 0);
  if (param.freeze_window > 0.0) {
    double cumulative = 0.0;
    for (int i = 0; i < ctrl_num; ++i) {
      cumulative += times(i);
      if (cumulative < param.freeze_window) {
        frozen[static_cast<std::size_t>(i)] = 1;
      }
    }
  }
  std::vector<int> opt_indices;
  opt_indices.reserve(static_cast<std::size_t>(ctrl_num));
  for (int i = 0; i < ctrl_num; ++i) {
    if (frozen[static_cast<std::size_t>(i)] == 0) {
      opt_indices.push_back(i);
    }
  }

  // ---- minco problem setup (head/tail at rest: the smoother binds both
  // ends, the planner's schedule carries the motion) ----
  minco::MINCO_S3NU minco;
  Eigen::Matrix<double, 2, 3> head_state = Eigen::Matrix<double, 2, 3>::Zero();
  Eigen::Matrix<double, 2, 3> tail_state = Eigen::Matrix<double, 2, 3>::Zero();
  head_state.col(0) = points.col(0);
  tail_state.col(0) = points.col(input_count - 1);
  minco.setConditions(head_state, tail_state, piece_num,
                      Eigen::Vector2d(param.weight_smooth, param.weight_smooth));

  // Optimization variable packing (x/y of the free control points).
  const int opt_num = static_cast<int>(opt_indices.size());
  Eigen::VectorXd x_opt(2 * std::max(opt_num, 0));
  for (int k = 0; k < opt_num; ++k) {
    const int i = opt_indices[static_cast<std::size_t>(k)];
    x_opt(k) = points(0, i + 1);
    x_opt(k + opt_num) = points(1, i + 1);
  }

  // Cost functional state (single-threaded LBFGS: shared scratch is fine).
  struct CostContext {
    minco::MINCO_S3NU* minco;
    const Eigen::Matrix2Xd* points;
    const Eigen::VectorXd* times;
    const std::vector<int>* opt_indices;
    const ayanami::rosemap::RosemapSnapshot* snapshot;
    double weight_pos;
    double safe_dist;
    double mu_l1;
    int opt_num;
    // Wall-clock + cancel guard (shared instance with the evaluate
    // callback: lbfgs passes ONE instance pointer to both).
    std::chrono::steady_clock::time_point deadline;
    std::atomic<bool>* canceled;
  };
  CostContext cost_context{
      &minco,      &points,           &times,          &opt_indices,
      snapshot.get(), param.weight_pos, param.safe_dist, param.mu_l1,
      opt_num,
      std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(budget)),
      &canceled};

  const auto cost_functional = [](void* instance_ptr, const Eigen::VectorXd& x,
                                  Eigen::VectorXd& g) -> double {
    auto* ctx = static_cast<CostContext*>(instance_ptr);
    const int ctrl = ctx->points->cols() - 2;

    // Unpack the full control-point set (frozen points keep input values).
    Eigen::Matrix2Xd in_ps = ctx->points->middleCols(1, ctrl);
    for (int k = 0; k < ctx->opt_num; ++k) {
      const int i = (*ctx->opt_indices)[static_cast<std::size_t>(k)];
      in_ps(0, i) = x(k);
      in_ps(1, i) = x(k + ctx->opt_num);
    }

    ctx->minco->setParameters(in_ps, *ctx->times);

    // Jerk energy + analytic gradient chain (minco).
    double energy = 0.0;
    Eigen::Matrix2Xd energy_grad = Eigen::Matrix2Xd::Zero(2, ctrl);
    Eigen::VectorXd energy_t_grad = Eigen::VectorXd::Zero(ctrl + 1);
    Eigen::MatrixX2d partial_by_coeffs;
    Eigen::VectorXd partial_by_times;
    ctx->minco->getEnergy(energy);
    ctx->minco->getEnergyPartialGradByCoeffs(partial_by_coeffs);
    ctx->minco->getEnergyPartialGradByTimes(partial_by_times);
    ctx->minco->propogateGrad(partial_by_coeffs, partial_by_times,
                              energy_grad, energy_t_grad);

    double cost_val = energy;
    Eigen::Matrix2Xd gradp = energy_grad;

    // ESDF position safety at the control points (rose attach_penalty_
    // functional semantics; Kahan summation against float drift).
    double cost_comp = 0.0;
    for (int i = 0; i < ctrl; ++i) {
      const Eigen::Vector2d p = in_ps.col(i);
      const auto query = ayanami::rosemap::sampleDistance(
          *ctx->snapshot, p.x(), p.y());
      // A failed/out-of-field query counts as violating, never as free.
      const double distance = query.status ==
                                      ayanami::rosemap::RosemapQueryStatus::kOk
                                  ? query.distance
                                  : -ctx->safe_dist;
      double f = 0.0;
      double df = 0.0;
      if (smoothedL1(ctx->safe_dist - distance, ctx->mu_l1, f, df)) {
        kahanSum(cost_val, cost_comp, ctx->weight_pos * f);
        const Eigen::Vector2d direction =
            query.gradient.norm() > 1e-6
                ? Eigen::Vector2d(-query.gradient.normalized())
                : Eigen::Vector2d::Zero();
        gradp.col(i).noalias() += ctx->weight_pos * df * direction;
      }
    }

    // Pack gradients back to the reduced variable vector.
    Eigen::VectorXd g_full(2 * ctrl);
    g_full.segment(0, ctrl) = gradp.row(0).transpose();
    g_full.segment(ctrl, ctrl) = gradp.row(1).transpose();
    g.resize(2 * ctx->opt_num);
    g.setZero();
    for (int k = 0; k < ctx->opt_num; ++k) {
      const int i = (*ctx->opt_indices)[static_cast<std::size_t>(k)];
      g(k) = g_full(i);
      g(k + ctx->opt_num) = g_full(i + ctrl);
    }
    return cost_val;
  };

  // LBFGS knobs (navi minco_optimizer semantics) + the wall-clock/cancel
  // guard through the progress callback.
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = param.lbfgs_mem_size;
  lbfgs_params.max_iterations = param.max_iterations;
  lbfgs_params.delta = param.opt_accuracy;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.past = 3;
  lbfgs_params.max_linesearch = 32;

  const auto progress_guard =
      [](void* instance_ptr, const Eigen::VectorXd&, const Eigen::VectorXd&,
         const double, const double, const int, const int) -> int {
    auto* ctx = static_cast<CostContext*>(instance_ptr);
    if (ctx->canceled->load()) {
      return 1;
    }
    if (std::chrono::steady_clock::now() >= ctx->deadline) {
      return 1;
    }
    return 0;
  };

  double min_cost = 0.0;
  int ret = 0;
  if (opt_num > 0) {
    ret = lbfgs::lbfgs_optimize(x_opt, min_cost, cost_functional, nullptr,
                                progress_guard, &cost_context, lbfgs_params);
  }

  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED, "smoothing canceled");
  }
  // LBFGS_CANCELED (progress != 0) also fires on the wall-clock budget: the
  // last iterate is still a usable candidate and flows into validation.
  const bool budget_exhausted = ret == lbfgs::LBFGS_CANCELED;
  const bool accepted_lbfgs =
      ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH || budget_exhausted;

  // ---- rebuild the full control-point set after optimization ----
  Eigen::Matrix2Xd in_ps(2, ctrl_num);
  for (int i = 0; i < ctrl_num; ++i) {
    in_ps(0, i) = points(0, i + 1);
    in_ps(1, i) = points(1, i + 1);
  }
  for (int k = 0; k < opt_num; ++k) {
    const int i = opt_indices[static_cast<std::size_t>(k)];
    in_ps(0, i) = x_opt(k);
    in_ps(1, i) = x_opt(k + opt_num);
  }
  minco.setParameters(in_ps, times);
  std::vector<Piece<5, 2>> pieces;
  minco.getPieces(pieces);
  if (pieces.empty()) {
    return ResultT<void>::failure(ErrorCode::SMOOTHING_FAILED,
                                  "minco produced no trajectory");
  }

  // ---- hard validation (navi validateTrajectory semantics) ----
  // Speed cap reference = the input path's own implied peak speed.
  double implied_peak = 0.0;
  for (int i = 0; i < piece_num; ++i) {
    if (times(i) > 1e-6) {
      implied_peak = std::max(implied_peak,
                              (points.col(i + 1) - points.col(i)).norm() /
                                  times(i));
    }
  }
  const double severe_speed =
      param.severe_scale * std::max(implied_peak, 1e-3);
  double t_offset = 0.0;
  bool valid = true;
  std::string invalid_reason;
  for (std::size_t p = 0; p < pieces.size() && valid; ++p) {
    const auto& piece = pieces[p];
    const double duration = piece.getDuration();
    for (double t = 0.0; t < duration && valid; t += param.validation_dt) {
      const Eigen::VectorXd position = piece.getPos(t);
      const Eigen::VectorXd velocity = piece.getVel(t);
      const Eigen::VectorXd acceleration = piece.getAcc(t);
      if (!position.allFinite() || !velocity.allFinite() ||
          !acceleration.allFinite()) {
        valid = false;
        invalid_reason = "non-finite trajectory state";
        break;
      }
      if (velocity.norm() > severe_speed) {
        valid = false;
        invalid_reason = "severe speed violation";
        break;
      }
      const auto query =
          ayanami::rosemap::sampleDistance(*snapshot, position(0), position(1));
      if (query.status == ayanami::rosemap::RosemapQueryStatus::kOk &&
          query.distance < 0.0) {
        valid = false;
        invalid_reason = "optimized trajectory collides";
        break;
      }
    }
    t_offset += duration;
  }

  if (!accepted_lbfgs) {
    // A hard LBFGS failure (not a budget/cancel exit) is a smoothing
    // failure: roll back (the BT's DegradeToRawPath branch also covers it).
    return ResultT<void>::failure(
        ErrorCode::SMOOTHING_FAILED,
        "lbfgs failed with code " + std::to_string(ret));
  }

  // Build the output path (dense stamped samples in the INPUT path's frame;
  // validation failure rolls back to the input path unchanged).
  const Eigen::Isometry3d path_from_field = field_from_path.inverse();
  nav_msgs::Path smoothed;
  smoothed.header.frame_id = path.header.frame_id;
  if (valid) {
    double stamp = 0.0;
    for (std::size_t p = 0; p < pieces.size(); ++p) {
      const auto& piece = pieces[p];
      const double duration = piece.getDuration();
      const int steps =
          std::max(1, static_cast<int>(std::ceil(duration / param.output_dt)));
      for (int s = 0; s < steps; ++s) {
        const double t = duration * static_cast<double>(s) /
                         static_cast<double>(steps);
        const Eigen::VectorXd position = piece.getPos(t);
        const Eigen::VectorXd velocity = piece.getVel(t);
        ayanami::geometry_msgs::PoseStamped pose;
        pose.header.frame_id = path.header.frame_id;
        const Eigen::Vector3d world =
            path_from_field * Eigen::Vector3d(position(0), position(1), 0.0);
        pose.pose.position = world;
        if (velocity.head<2>().norm() > 1e-6) {
          pose.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(
              std::atan2(velocity(1), velocity(0)), Eigen::Vector3d::UnitZ()));
        }
        pose.header.stamp = ayanami::Timestamp(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(stamp)));
        smoothed.poses.push_back(std::move(pose));
        stamp += duration / static_cast<double>(steps);
      }
    }
    // Anchor the final pose exactly at the (transformed) input goal.
    ayanami::geometry_msgs::PoseStamped tail;
    tail.header.frame_id = path.header.frame_id;
    tail.pose.position =
        path_from_field * Eigen::Vector3d(points(0, input_count - 1),
                                          points(1, input_count - 1), 0.0);
    tail.pose.orientation = path.poses.back().pose.orientation;
    tail.header.stamp = ayanami::Timestamp(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(stamp)));
    smoothed.poses.push_back(std::move(tail));
  }

  auto out = ResultT<void>::success();
  if (valid) {
    out.message = budget_exhausted
                      ? "smoothed within budget (lbfgs stopped at deadline)"
                      : "smoothed";
  } else {
    // Rollback: keep the input path, still report SUCCESS (smoothing is an
    // enhancement; a pseudo-collision of the optimizer must not send the
    // navigation into the recovery ladder).
    out.message = "smoothing rolled back (" + invalid_reason +
                  "); input path kept";
  }
  if (valid) {
    path = std::move(smoothed);
  }
  // Rollback (validation failed): the input path is left untouched.
  return out;
}

}  // namespace modules
}  // namespace ayanami
