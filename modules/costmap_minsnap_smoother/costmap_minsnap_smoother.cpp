// modules/costmap_minsnap_smoother -- minimum-snap smoothing over a square
// safety corridor (tdt-nav-kit MinimumSnap semantics). See the header for
// the backend split and the map-access discipline.

#include "costmap_minsnap_smoother.hpp"

#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace ayanami {
namespace modules {

namespace {

using minsnap::ControlPoint;
using minsnap::SolveProblem;

constexpr double k_epsilon = 1e-9;

// Uniformly subsamples the polyline to at most max_points vertices (first
// and last always kept).
std::vector<Eigen::Vector2d> subsample(
    const std::vector<Eigen::Vector2d>& points, int max_points) {
  if (max_points <= 0 ||
      static_cast<int>(points.size()) <= max_points) {
    return points;
  }
  std::vector<Eigen::Vector2d> out;
  out.reserve(static_cast<std::size_t>(max_points));
  const double step =
      static_cast<double>(points.size() - 1) / (max_points - 1);
  for (int i = 0; i < max_points; ++i) {
    out.push_back(points[static_cast<std::size_t>(
        std::min<double>(std::round(i * step), points.size() - 1))]);
  }
  out.front() = points.front();
  out.back() = points.back();
  return out;
}

}  // namespace

void CostmapMinSnapSmoother::configureParameters(
    const ayanami::ModuleContext& context) {
  // Parameter loading, separated from configuration: the loader calls this
  // right after a successful load, and every value lands in the module's
  // own CostmapMinSnapSmootherParam member (whose field initializers are
  // the defaults for keys the config omits).
  const Config& file = context.config;
  const std::string& section = context.name;
  param.polynomial_order = file.param<int>(section, "polynomial_order",
                                           param.polynomial_order);
  param.derivative_order = file.param<int>(section, "derivative_order",
                                           param.derivative_order);
  param.max_speed = file.param<double>(section, "max_speed", param.max_speed);
  param.max_acc = file.param<double>(section, "max_acc", param.max_acc);
  param.max_corridor_range = file.param<double>(
      section, "max_corridor_range", param.max_corridor_range);
  param.corridor_shrink = file.param<double>(section, "corridor_shrink",
                                             param.corridor_shrink);
  param.collision_iterations = file.param<int>(section, "collision_iterations",
                                               param.collision_iterations);
  param.normalized_time = file.param<bool>(section, "normalized_time",
                                           param.normalized_time);
  param.backend = file.param<std::string>(section, "backend", param.backend);
  param.max_waypoints = file.param<int>(section, "max_waypoints",
                                        param.max_waypoints);
}

void CostmapMinSnapSmoother::configure(
    const ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // Direct-construction callers (tests, or a host that builds the object
  // itself) never go through the loader's configureParameters call, so bind
  // the parameters here as well. Re-reading the same keys is idempotent.
  configureParameters(context);
  map_context = std::move(map);
}

bool CostmapMinSnapSmoother::cancel() {
  canceled.store(true);
  return true;
}

bool CostmapMinSnapSmoother::solveIterated(
    minsnap::SolveProblem& problem, std::vector<corridor::Box>& corridor,
    bool use_osqp, const std::function<bool(double, double)>& world_blocked,
    double cell_size,
    const std::optional<std::chrono::steady_clock::time_point>& deadline,
    std::vector<Eigen::Vector2d>& samples_out) {

  auto solveOnce = [&](const std::vector<double>& times,
                       const std::vector<ControlPoint>& points,
                       minsnap::SegmentCoefficients& coefficients) {
    if (use_osqp) {
      return minsnap::solveOsqpCorridor(problem, corridor, coefficients);
    }
    return minsnap::solveClosedForm(problem, coefficients);
  };

  for (int round = 0; round < param.collision_iterations; ++round) {
    if (canceled.load()) {
      return false;
    }
    // Round-entry deadline: each solveOnce (OSQP especially) is one
    // uninterruptible block -- a new one never STARTS past the budget.
    if (deadline.has_value() &&
        std::chrono::steady_clock::now() >= *deadline) {
      return false;
    }
    minsnap::SegmentCoefficients coefficients;
    if (!solveOnce(problem.times, problem.points, coefficients)) {
      return false;
    }

    const int segments = static_cast<int>(problem.times.size());
    samples_out.clear();
    std::vector<int> bad;
    for (int i = 0; i < segments; ++i) {
      if (canceled.load()) {
        return false;
      }
      const std::vector<Eigen::Vector2d> curve =
          minsnap::sampleSegment(coefficients, problem.order, problem.times,
                                 i, problem.normalized_time);
      bool finite = static_cast<int>(curve.size()) > 1;
      for (const auto& p : curve) {
        finite = finite && p.allFinite();
      }
      if (!finite) {
        samples_out.clear();
        return false;
      }
      bool hit = false;
      for (std::size_t j = 1; j < curve.size(); ++j) {
        hit = hit || corridor::segmentBlocked(world_blocked, curve[j - 1],
                                              curve[j], cell_size);
      }
      if (hit) {
        bad.push_back(i);
      }
      // Append without duplicating the joint vertex.
      const bool last = i + 1 == segments;
      samples_out.insert(samples_out.end(), curve.begin(),
                         last ? curve.end() : curve.end() - 1);
    }
    if (bad.empty()) {
      return true;
    }
    if (round + 1 == param.collision_iterations ||
        (deadline.has_value() &&
         std::chrono::steady_clock::now() >= *deadline)) {
      return false;
    }

    // tdt repair alternation: even rounds insert midpoints of colliding
    // segments; odd rounds shrink (corridor backend) or time-compress
    // (closed-form backend) the colliding segments.
    if (round % 2 == 0) {
      const std::vector<int> inserted =
          minsnap::insertMidpoints(problem.times, problem.points, bad);
      if (use_osqp) {
        for (int index : inserted) {
          corridor::Box bound;
          const Eigen::Vector2d& p = problem.points[index].xy;
          if (corridor::computeBound(world_blocked, p.x(), p.y(),
                                     param.max_corridor_range, cell_size,
                                     param.corridor_shrink, bound)) {
            corridor.insert(corridor.begin() + index, bound);
          } else {
            corridor.insert(corridor.begin() + index,
                            {p.x(), p.y(), p.x(), p.y()});
          }
        }
      } else {
        for (int index : inserted) {
          corridor.insert(corridor.begin() + index,
                          {problem.points[index].xy.x(),
                           problem.points[index].xy.y(),
                           problem.points[index].xy.x(),
                           problem.points[index].xy.y()});
        }
      }
    } else if (use_osqp) {
      for (int i : bad) {
        for (int index : {i, i + 1}) {
          if (index >= static_cast<int>(corridor.size())) {
            continue;
          }
          const double len = std::max(corridor[index][2] - corridor[index][0],
                                      corridor[index][3] - corridor[index][1]);
          if (len > 0.0) {
            corridor[index] = corridor::shrinkBound(
                corridor[index], len * 0.5, {problem.points[index].xy});
          }
        }
      }
    } else {
      for (int i : bad) {
        problem.times[i] *= 0.8;
      }
    }
  }
  return false;
}

ResultT<void> CostmapMinSnapSmoother::smoothPath(
    ayanami::nav_msgs::Path& path, double max_duration) {
  if (!map_context) {
    return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                  "CostmapMinSnapSmoother has no costmap");
  }
  canceled.store(false);
  const ayanami::nav_msgs::Path original = path;
  // The caller's budget covers the WHOLE call (initial corridor build
  // included), so the deadline starts here rather than inside solveIterated:
  // a budget spent before the solve begins must degrade, never run a
  // partial solve. Negative max_duration = untimed (established semantics).
  std::optional<std::chrono::steady_clock::time_point> deadline;
  if (max_duration >= 0.0) {
    deadline = std::chrono::steady_clock::now() +
               std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                   std::chrono::duration<double>(max_duration));
  }
  const auto budget_expired = [&deadline]() {
    return deadline.has_value() &&
           std::chrono::steady_clock::now() >= *deadline;
  };

  // Global held copy + one short-lock local snapshot per call; every
  // corridor/collision query below runs on this immutable pair.
  const std::uint64_t global_gen = map_context->globalGeneration();
  if (!have_global_copy || global_gen != global_copy_generation) {
    if (map_context->snapshotGlobal(global_copy)) {
      global_copy_generation = global_copy.generation;
      have_global_copy = true;
    }
  }
  ayanami::costmap::FusedMapView view;
  view.global = &global_copy;
  const bool global_valid = have_global_copy && global_copy.valid();
  bool local_valid = false;
  if (map_context->snapshotLocal(local_snapshot) && local_snapshot.valid()) {
    if (map_context->localFromGlobal(view.local_from_global)) {
      view.local = &local_snapshot;
      local_valid = true;
    }
  }
  // Without any usable grid the smoother degrades to unchecked solving
  // (the BT-level IsPathValid still covers the path).
  const bool map_available = global_valid || local_valid;
  const std::function<bool(double, double)> world_blocked =
      [&view, map_available](double x, double y) {
        if (!map_available) {
          return false;
        }
        // blockedAt already applies the >= 253 blocking convention.
        return view.blockedAt(x, y);
      };

  if (path.poses.size() < 2) {
    return ResultT<void>::success();
  }

  // Waypoints: subsampled polyline; the terminal velocity is zero (the
  // robot stops at the goal), the initial velocity is left free (tdt
  // default), interior points are pinned (closed form) or corridor-boxed
  // (OSQP).
  std::vector<Eigen::Vector2d> waypoints;
  waypoints.reserve(path.poses.size());
  for (const auto& pose : path.poses) {
    waypoints.emplace_back(pose.pose.position.x(), pose.pose.position.y());
  }
  waypoints = subsample(waypoints, param.max_waypoints);

  const std::vector<double> times = minsnap::trapezoidalTimeAllocation(
      waypoints, param.max_speed, param.max_acc);
  SolveProblem problem;
  problem.order = param.polynomial_order;
  problem.derivative_order = param.derivative_order;
  problem.normalized_time = param.normalized_time;
  problem.times = times;
  problem.points.resize(waypoints.size());
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    problem.points[i].xy = waypoints[i];
    problem.points[i].corridor = {waypoints[i].x(), waypoints[i].y(),
                                  waypoints[i].x(), waypoints[i].y()};
  }
  // Pin both ends: the trajectory must start at the path start (the
  // robot pose) and stop at the goal; velocity is zero at both ends (the
  // interface carries no timing -- velocity continuity is a track-level
  // concern, see the module docs).
  problem.points.front().has_velocity = true;
  problem.points.back().has_velocity = true;
  if (!minsnap::validTimes(problem.times, problem.points.size(),
                           problem.order, problem.derivative_order)) {
    path = original;
    auto result = ResultT<void>::success();
    result.message = "invalid time allocation; kept the original path";
    return result;
  }

  const bool use_osqp = param.backend == "osqp_corridor";
  const double cell_size =
      global_valid ? global_copy.resolution
                   : (local_valid ? local_snapshot.resolution : 0.05);

  // Initial corridors (OSQP backend): snap the anchor to its cell center so
  // the expansion stays raster-aligned.
  std::vector<corridor::Box> corridor(problem.points.size());
  for (std::size_t i = 0; i < problem.points.size(); ++i) {
    if (canceled.load() || budget_expired()) {
      // Budget spent (or cancel requested) before the solve even started:
      // graceful degradation, same contract as a failed solve below.
      path = original;
      auto result = ResultT<void>::success();
      result.message =
          "smoothing budget expired during corridor build; kept the original path";
      return result;
    }
    corridor::Box bound;
    if (use_osqp && map_available) {
      const double ax = std::floor((problem.points[i].xy.x() -
                                    view.global->origin_x) /
                                   cell_size) *
                            cell_size +
                        view.global->origin_x + 0.5 * cell_size;
      const double ay = std::floor((problem.points[i].xy.y() -
                                    view.global->origin_y) /
                                   cell_size) *
                            cell_size +
                        view.global->origin_y + 0.5 * cell_size;
      if (!corridor::computeBound(world_blocked, ax, ay,
                                  param.max_corridor_range, cell_size,
                                  param.corridor_shrink, bound)) {
        bound = {problem.points[i].xy.x(), problem.points[i].xy.y(),
                 problem.points[i].xy.x(), problem.points[i].xy.y()};
      }
      corridor[i] = bound;
    }
  }

  std::vector<Eigen::Vector2d> samples;
  const bool solved = solveIterated(problem, corridor, use_osqp,
                                    world_blocked, cell_size, deadline,
                                    samples);
  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED, "Smoothing canceled");
  }
  if (!solved || samples.size() < 2) {
    // Graceful degradation, NOT a failure:
    // the caller keeps following the path the planner already produced and
    // validated; a pure smoothing artifact must not enter the BT recovery
    // ladder.
    path = original;
    auto result = ResultT<void>::success();
    result.message = "minimum-snap solve failed; kept the original path";
    return result;
  }

  // Write back: sampled trajectory in the ORIGINAL path frame; the exact
  // original end poses (position AND orientation) are preserved so the
  // controller-facing goal state is untouched.
  ayanami::nav_msgs::Path smoothed;
  smoothed.header = original.header;
  smoothed.poses.reserve(samples.size());
  for (const auto& p : samples) {
    auto pose = ayanami::geometry_msgs::PoseStamped::fromIsometry(
        smoothed.header.frame_id.empty() ? "map"
                                         : smoothed.header.frame_id,
        Eigen::Isometry3d::Identity());
    pose.header.frame_id = smoothed.header.frame_id;
    pose.pose.position = Eigen::Vector3d(p.x(), p.y(), 0.0);
    smoothed.poses.push_back(std::move(pose));
  }
  smoothed.poses.front().pose = original.poses.front().pose;
  smoothed.poses.back().pose = original.poses.back().pose;
  for (std::size_t i = 1; i + 1 < smoothed.poses.size(); ++i) {
    const Eigen::Vector3d d = smoothed.poses[i + 1].pose.position -
                              smoothed.poses[i - 1].pose.position;
    const double yaw = std::atan2(d.y(), d.x());
    smoothed.poses[i].pose.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  }

  path = std::move(smoothed);
  auto result = ResultT<void>::success();
  result.message = "minimum-snap smoothing succeeded";
  return result;
}

}  // namespace modules
}  // namespace ayanami
