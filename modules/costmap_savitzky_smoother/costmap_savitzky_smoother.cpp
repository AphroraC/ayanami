// modules/costmap_savitzky_smoother -- 7-point Savitzky-Golay path smoothing.
//
// Pure C++ (zero ROS): collision checks run on a lock-held zero-copy map read via CostmapView (global master grid).
// The core is the 7-point SG convolution (nav2 SavitzkyGolaySmoother semantics):
// - coefficients [-2, 3, 6, 7, 6, 3, -2] / 21; end points fixed; interior points convolved over the window
// - places with less than a full window at the boundaries use replicated padding (clamped to the end points)
// - do_refinement: applies refinement_num more filter rounds to the result (bounded by max_duration)
// - collision check: after smoothing, queries the local costmap point by point; any cost >=
// INSCRIBED_INFLATED_OBSTACLE (lethal from 253 on) rolls back to the original

#include "costmap_savitzky_smoother.hpp"

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

}  // namespace


void CostmapSavitzkySmoother::configure(
    const PluginContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // Parameters are read from the PluginContext-injected own configuration (the plugin does not know the global config layout).
  const Config& file = context.config;
  // Param section convention: the section key = the module json "name"
  // field, delivered as PluginContext::name.
  const std::string& section = context.name;
  param.do_refinement =
      file.param<bool>(section, "do_refinement",
                       param.do_refinement);
  param.refinement_num =
      file.param<int>(section, "refinement_num",
                      param.refinement_num);
  map_context = std::move(map);
}

bool CostmapSavitzkySmoother::cancel() {
  canceled.store(true);
  return true;
}

ResultT<void> CostmapSavitzkySmoother::smoothPath(
    ayanami::nav_msgs::Path& path, double max_duration) {
  // Collision checking needs the map (family read contract: smoother reads
  // the LOCAL map): shared lock held for the whole smoothing call (<10 ms),
  // zero copy.
  std::optional<ayanami::costmap::CostmapView> costmap;
  if (!map_context) {
    return ResultT<void>::failure(
        ErrorCode::MAP_UNAVAILABLE,
        "CostmapSavitzkySmoother has no costmap module");
  }
  costmap = map_context->lockLocal();
  const ayanami::nav_msgs::Path original = path;
  canceled.store(false);

  // Empty path or fewer than 7 points: no window to smooth, return SUCCESS directly with path unchanged.
  if (path.poses.size() < 7) {
    auto result = ResultT<void>::success();
    result.message = "path too short to smooth";
    return result;
  }

  const auto t0 = std::chrono::steady_clock::now();
  const auto elapsed = [&t0]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
        .count();
  };
  // max_duration < 0 means unlimited time.
  const bool timed = max_duration >= 0.0;

  // Budget exhausted before starting: SUCCESS + partial smoothing (= none).
  if (timed && elapsed() > max_duration) {
    return ResultT<void>::success();
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

  // Collision check: after smoothing, query the lock-held read view point by point; on a lethal hit roll back to the original.
  const costmap_2d::Costmap2D* cm = costmap ? costmap->grid() : nullptr;
  if (cm) {
    for (const auto& p : path.poses) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!cm->worldToMap(p.pose.position.x(), p.pose.position.y(),
                          mx, my)) {
        continue;
      }
      if (cm->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        path = original;
        auto result = ResultT<void>::success();
        result.message = "collision detected; rolled back to original path";
        return result;
      }
    }
  }

  // Identical to the original also degenerates to SUCCESS.
  return ResultT<void>::success();
}

}  // namespace modules
}  // namespace ayanami
