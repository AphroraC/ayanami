// modules/costmap_savitzky_smoother -- 7-point Savitzky-Golay path smoothing.
//
// Pure C++ (zero ROS): collision checks run on the smoother's OWN map
// copies, kept fresh by subscribing to NavigationCallbacks::on_costmap_raw
// (the host emits a lossless full-grid copy under the map's shared lock,
// only when the content changed). No live-map lock is ever held by the
// smoother: a long smoothing pass delays the map writer by nothing.
// The core is the 7-point SG convolution (nav2 SavitzkyGolaySmoother semantics):
// - coefficients [-2, 3, 6, 7, 6, 3, -2] / 21; end points fixed; interior points convolved over the window
// - places with less than a full window at the boundaries use replicated padding (clamped to the end points)
// - do_refinement: applies refinement_num more filter rounds to the result (bounded by max_duration)
// - collision check: after smoothing, the path is queried point by point on
// both copies; any cost >= INSCRIBED_INFLATED_OBSTACLE (lethal from 253 on)
// rolls back to the original and still reports SUCCESS -- the rollback is a
// graceful degradation, not a failure (see the call site).
// Frames: the path is expressed in its header frame (production: the global
// costmap frame), each copy in its own grid frame -- the grid<-path
// transform is resolved ONCE per smooth call (no per-point TF queries) and
// the returned path keeps its original frame.

#include "costmap_savitzky_smoother.hpp"

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace ayanami {
namespace modules {

namespace {

// 7-point SG filter coefficients (nav2_smoother §2, normalized /21).
constexpr std::array<double, 7> k_filter = {-2.0 / 21.0, 3.0 / 21.0, 6.0 / 21.0,
                                           7.0 / 21.0,  6.0 / 21.0, 3.0 / 21.0,
                                           -2.0 / 21.0};

// Replicated-padding window index: out-of-range values clamp to [0, n-1].
std::size_t clampIndex(std::size_t i, int k, std::size_t n) {
  const long long idx = static_cast<long long>(i) + k;
  return static_cast<std::size_t>(
      std::clamp(idx, 0LL, static_cast<long long>(n - 1)));
}

// Applies one SG convolution round to interior points (1..n-2); end points are fixed. Reads original values, writes once.
void applySmoothPass(std::vector<ayanami::geometry_msgs::PoseStamped>& poses) {
  const std::size_t n = poses.size();
  std::vector<Eigen::Vector3d> smoothed(n);
  smoothed.front() = poses.front().pose.position;
  smoothed.back() = poses.back().pose.position;

  for (std::size_t i = 1; i + 1 < n; ++i) {
    Eigen::Vector3d val = Eigen::Vector3d::Zero();
    for (int k = -3; k <= 3; ++k) {
      const std::size_t j = clampIndex(i, k, n);
      val += k_filter[static_cast<std::size_t>(k + 3)] *
             poses[j].pose.position;
    }
    smoothed[i] = val;
  }

  for (std::size_t i = 1; i + 1 < n; ++i) {
    poses[i].pose.position = smoothed[i];
  }
}

// Aligns interior point orientations to the tangent direction (end points keep their original orientations).
void updateOrientations(std::vector<ayanami::geometry_msgs::PoseStamped>& poses) {
  for (std::size_t i = 1; i + 1 < poses.size(); ++i) {
    const Eigen::Vector3d d = poses[i + 1].pose.position -
                              poses[i - 1].pose.position;
    const double yaw = std::atan2(d.y(), d.x());
    poses[i].pose.orientation =
        Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  }
}

// Position identity (memoization key): size + per-point isApprox; header
// frames must match so the cached output is frame-compatible.
bool pathEquals(const ayanami::nav_msgs::Path& lhs,
                const ayanami::nav_msgs::Path& rhs) {
  if (lhs.poses.size() != rhs.poses.size() ||
      lhs.header.frame_id != rhs.header.frame_id) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.poses.size(); ++i) {
    if (!lhs.poses[i].pose.position.isApprox(rhs.poses[i].pose.position)) {
      return false;
    }
  }
  return true;
}

}  // namespace



void CostmapSavitzkySmoother::configureParameters(const ayanami::ModuleContext& context) {
  // Parameter loading, separated from configuration: the loader calls this
  // right after a successful load, and every value lands in the module's own
  // CostmapSavitzkySmootherParam member (whose field initializers are the defaults for keys the
  // config omits).
  // Parameters are read from the ModuleContext-injected own configuration (the module does not know the global config layout).
  const Config& file = context.config;
  // Param section convention: the section key = the module json "name"
  // field, delivered as ModuleContext::name.
  const std::string& section = context.name;
  param.do_refinement =
      file.param<bool>(section, "do_refinement",
                       param.do_refinement);
  param.refinement_num =
      file.param<int>(section, "refinement_num",
                      param.refinement_num);
}

void CostmapSavitzkySmoother::configure(
    const ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // Direct-construction callers (tests, or a host that builds the object
  // itself) never go through the loader's configureParameters call, so bind
  // the parameters here as well. Re-reading the same keys from the same
  // config is idempotent.
  configureParameters(context);
  map_context = std::move(map);

  // Map copies arrive through the global raw slot (idempotent: the loader
  // and direct construction may both call configure).
  if (!subscriptions_active) {
    global_subscription =
        ayanami::interfaces::NavigationCallbacks::on_costmap_raw.subscribe(
            [this](const ayanami::nav_msgs::RawCostmap& raw) {
              onRawCostmap(raw);
            });
    local_subscription =
        ayanami::interfaces::NavigationCallbacks::on_costmap_raw.subscribe(
            [this](const ayanami::nav_msgs::RawCostmap& raw) {
              onRawCostmap(raw);
            });
    subscriptions_active = true;
  }
}

void CostmapSavitzkySmoother::onRawCostmap(
    const ayanami::nav_msgs::RawCostmap& raw) {
  std::lock_guard<std::mutex> lock(snapshots_mutex);
  if (raw.name == "global_costmap") {
    global_snapshot.data = raw;
    global_snapshot.valid = true;
  } else if (raw.name == "local_costmap") {
    local_snapshot.data = raw;
    local_snapshot.valid = true;
  }
}

CostmapSavitzkySmoother::RawSnapshot CostmapSavitzkySmoother::snapshot(
    const std::string& name) const {
  std::lock_guard<std::mutex> lock(snapshots_mutex);
  if (name == "global_costmap") {
    return global_snapshot;
  }
  if (name == "local_costmap") {
    return local_snapshot;
  }
  return RawSnapshot{};
}

bool CostmapSavitzkySmoother::cancel() {
  canceled.store(true);
  return true;
}

ResultT<void> CostmapSavitzkySmoother::smoothPath(
    ayanami::nav_msgs::Path& path, double max_duration) {
  if (!map_context) {
    return ResultT<void>::failure(
        ErrorCode::MAP_UNAVAILABLE,
        "CostmapSavitzkySmoother has no costmap module");
  }
  canceled.store(false);

  // The smoother's own map copies (slot-fed; no live-map locks).
  const RawSnapshot global = snapshot("global_costmap");
  const RawSnapshot local = snapshot("local_costmap");
  const uint64_t global_revision =
      global.valid ? global.data.revision : k_no_map;
  const uint64_t local_revision = local.valid ? local.data.revision : k_no_map;

  // Memoization: identical input path on unchanged maps -> the cached
  // output IS the answer (write it through; no smoothing work, no map
  // queries). A changed path or any map revision advance re-smooths.
  if (has_cache && pathEquals(cached_input, path) &&
      global_revision == cached_global_revision &&
      local_revision == cached_local_revision) {
    path = cached_output;
    auto result = ResultT<void>::success();
    result.message = "smoothing cache hit (input path and maps unchanged)";
    return result;
  }

  const ayanami::nav_msgs::Path original = path;

  // 1. Core smoothing (no map access): the SG convolution needs no grid.
  smoothInPlace(path, max_duration);

  // 2. Collision check on BOTH timescales (family read contract: smoother =
  // global+local collision checking) against the smoother's own copies.
  bool collision = false;
  if (global.valid) {
    collision = collidesWithSnapshot(path, global);
  }
  if (!collision && local.valid) {
    collision = collidesWithSnapshot(path, local);
  }
  if (collision) {
    // Graceful degradation, NOT a failure: restoring the original path
    // leaves the caller following the path the planner already produced
    // and validated. Reporting a failure here would send a pure
    // smoothing artifact into the recovery ladder (spin/wait/back-up)
    // instead of simply walking the raw path.
    path = original;
    has_cache = true;
    cached_input = original;
    cached_output = path;
    cached_global_revision = global_revision;
    cached_local_revision = local_revision;
    auto result = ResultT<void>::success();
    result.message = "map collision detected; rolled back to original path";
    return result;
  }

  has_cache = true;
  cached_input = original;
  cached_output = path;
  cached_global_revision = global_revision;
  cached_local_revision = local_revision;

  // Identical to the original also degenerates to SUCCESS.
  return ResultT<void>::success();
}

void CostmapSavitzkySmoother::smoothInPlace(ayanami::nav_msgs::Path& path,
                                            double max_duration) {
  // Empty path or fewer than 7 points: no window to smooth, nothing to do.
  if (path.poses.size() < 7) {
    return;
  }

  const auto t0 = std::chrono::steady_clock::now();
  const auto elapsed = [&t0]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
        .count();
  };
  // max_duration < 0 means unlimited time.
  const bool timed = max_duration >= 0.0;

  // Budget exhausted before starting: no smoothing.
  if (timed && elapsed() > max_duration) {
    return;
  }

  // Nominal one filter round (core smoothing).
  applySmoothPass(path.poses);

  // Refinement rounds (early stop under the time budget / cancel constraint; the already-smoothed part is kept).
  if (param.do_refinement) {
    for (int r = 0; r < param.refinement_num; ++r) {
      if (canceled.load() || (timed && elapsed() >= max_duration)) {
        break;
      }
      applySmoothPass(path.poses);
    }
  }

  updateOrientations(path.poses);
}

bool CostmapSavitzkySmoother::collidesWithSnapshot(
    const ayanami::nav_msgs::Path& path,
    const RawSnapshot& snapshot) const {
  if (!snapshot.valid) {
    return false;
  }
  const ayanami::nav_msgs::RawCostmap& cm = snapshot.data;
  if (cm.costs.empty()) {
    return false;
  }
  // The path frame and the grid frame may differ (map vs odom): the
  // grid<-path transform is resolved ONCE per call and applied to the query
  // points.
  const std::string& grid_frame = cm.frame_id;
  const std::string path_frame = path.header.frame_id.empty()
                                     ? (map_context ? map_context->globalFrame()
                                                    : std::string{})
                                     : path.header.frame_id;
  Eigen::Isometry3d grid_from_path = Eigen::Isometry3d::Identity();
  const bool need_transform =
      !path_frame.empty() && !grid_frame.empty() && path_frame != grid_frame;
  if (need_transform) {
    if (!ayanami::components::RobotStateController::instance()
             .lookupTransform(grid_frame, path_frame,
                              ayanami::Timestamp::latest(), grid_from_path,
                              0.0)) {
      // A missing frame transform for this grid is a degraded check, not a
      // collision: the other grid (and the BT-level IsPathValid) still
      // covers the path.
      return false;
    }
  }
  for (const auto& p : path.poses) {
    const Eigen::Vector3d query =
        need_transform ? (grid_from_path * p.pose.position) : p.pose.position;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!cm.worldToMap(query.x(), query.y(), mx, my)) {
      continue;
    }
    if (cm.getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return true;
    }
  }
  return false;
}

}  // namespace modules
}  // namespace ayanami
