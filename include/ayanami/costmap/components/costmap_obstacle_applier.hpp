#pragma once

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <spdlog/spdlog.h>

#include <costmap/costmap_wrapper.hpp>
#include <costmap/simple_obstacle_layer.hpp>

#include "ayanami/costmap/core/costmap_observation.hpp"
#include "ayanami/utility/logging.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief Applies one resolved observation to the local map's obstacle layer
 * (the static-composition successor of the former obstacle layer plugin's
 * observe()).
 *
 * Runs under the target map's WRITE lock (the sole map writer resolved the
 * stamp-time TF and robot pose already) and performs: hit filtering in the
 * map frame (height window judged after the map-frame transform, marking
 * range window from the sensor origin) -> per-endpoint raytrace clearing ->
 * marking -> footprint clearing LAST (nav2 semantics: chassis hits lose to
 * the clear, so the robot can never mark itself). A frame with no ray
 * endpoints clears nothing; retired marks age out via decay_time.
 *
 * The filtering policy lives here, not in the map package: the policy keys
 * are observation-source parameters (ObservationSourceConfig) owned by the
 * host config, and the warn paths route through the ayanami "costmap"
 * logger.
 */
class CostmapObstacleApplier {
 public:
  using Ptr = std::shared_ptr<CostmapObstacleApplier>;

  explicit CostmapObstacleApplier(
      std::shared_ptr<costmap_2d::SimpleObstacleLayer> layer,
      bool footprint_clearing)
      : layer_(std::move(layer)), footprint_clearing_(footprint_clearing) {}

  const std::shared_ptr<costmap_2d::SimpleObstacleLayer>& layer() const {
    return layer_;
  }

  void apply(const MapObservation& observation) {
    const auto* source = observation.source;
    auto* master = layer_->layeredCostmap();
    if (source == nullptr || master == nullptr ||
        observation.points == nullptr ||
        observation.clearing_points == nullptr) {
      return;
    }
    auto* grid = master->getCostmap();
    if (grid == nullptr) {
      return;
    }
    const Eigen::Isometry3d& t_frame_sensor = observation.t_frame_sensor;
    const Eigen::Vector3d sensor_origin = t_frame_sensor.translation();

    // Source-driven hit filtering: height window (z judged in the MAP frame
    // -- points are transformed FIRST, then filtered; sensor-frame z says
    // nothing about height above the ground plane under mounting tilt) and
    // marking range window from the sensor origin (nav2 semantics).
    // height_relative_to_robot shifts the window onto the robot's current
    // height (ramps under the robot drop out of the lower edge; a missing
    // robot z degrades to the absolute window).
    double z_min = source->min_obstacle_height;
    double z_max = source->max_obstacle_height;
    if (source->height_relative_to_robot &&
        std::isfinite(observation.robot_z)) {
      z_min += observation.robot_z;
      z_max += observation.robot_z;
    }
    std::vector<Eigen::Vector2d> world;
    world.reserve(observation.points->size());
    if (source->marking) {
      for (const auto& p : *observation.points) {
        const Eigen::Vector3d w = t_frame_sensor * p;
        if (w.z() < z_min || w.z() > z_max) {
          continue;
        }
        const double range = (w.head<2>() - sensor_origin.head<2>()).norm();
        if (range < source->obstacle_min_range ||
            range > source->obstacle_max_range) {
          continue;
        }
        world.emplace_back(w.x(), w.y());
      }
    }

    // Ray endpoints are consumed as-is (skipped below raytrace_min_range,
    // clamped to raytrace_max_range). A frame with NO endpoints (e.g. an
    // empty obstacle cloud) clears nothing: retired marks age out via
    // decay_time instead of being wiped wholesale.
    if (source->clearing) {
      for (const auto& endpoint : *observation.clearing_points) {
        const double range = endpoint.head<2>().norm();
        if (range < source->raytrace_min_range) {
          continue;
        }
        Eigen::Vector3d clamped = endpoint;
        if (range > source->raytrace_max_range) {
          clamped = endpoint * (source->raytrace_max_range / range);
        }
        const Eigen::Vector3d world_end = t_frame_sensor * clamped;
        layer_->raytraceFreespace(*grid, sensor_origin.x(), sensor_origin.y(),
                                  world_end.x(), world_end.y());
      }
    }

    if (source->marking) {
      // Points outside this map's bounds cannot be marked (worldToMap
      // fails); count them and say so -- the usual fix is a larger map size
      // or a smaller obstacle_max_range.
      std::size_t outside_window = 0;
      unsigned int probe_mx = 0;
      unsigned int probe_my = 0;
      for (const auto& w : world) {
        if (!grid->worldToMap(w.x(), w.y(), probe_mx, probe_my)) {
          ++outside_window;
          continue;
        }
        layer_->markObstacles(*grid, w.x(), w.y());
      }
      if (outside_window > 0) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_outside_warn_at_ >= std::chrono::seconds(5)) {
          last_outside_warn_at_ = now;
          SPDLOG_LOGGER_WARN(
            ayanami::costmapLogger(),
            "obstacle layer '{}': {} of {} marked points fell outside the "
            "'{}' map ({:.0f}x{:.0f} m); enlarge the map size or reduce "
            "obstacle_max_range",
            layer_->getName(),
            outside_window,
            world.size(),
            master->getGlobalFrameID(),
            grid->getSizeInMetersX(),
            grid->getSizeInMetersY());
        }
      }
    }

    // Footprint clearing LAST (nav2 semantics): hits from the robot's own
    // chassis lose to the clear, so the robot can never mark itself with
    // ghost obstacles right under it.
    if (footprint_clearing_ && observation.has_robot_pose) {
      const auto& footprint = master->getFootprint();
      if (!footprint.empty()) {
        const double robot_x = observation.robot_pose_xy_yaw.x();
        const double robot_y = observation.robot_pose_xy_yaw.y();
        const double robot_yaw = observation.robot_pose_xy_yaw.z();
        double fp_min_x = std::numeric_limits<double>::max();
        double fp_min_y = std::numeric_limits<double>::max();
        double fp_max_x = std::numeric_limits<double>::lowest();
        double fp_max_y = std::numeric_limits<double>::lowest();
        const double cos_yaw = std::cos(robot_yaw);
        const double sin_yaw = std::sin(robot_yaw);
        for (const auto& point : footprint) {
          const double wx = robot_x + point.x * cos_yaw - point.y * sin_yaw;
          const double wy = robot_y + point.x * sin_yaw + point.y * cos_yaw;
          fp_min_x = std::min(fp_min_x, wx);
          fp_min_y = std::min(fp_min_y, wy);
          fp_max_x = std::max(fp_max_x, wx);
          fp_max_y = std::max(fp_max_y, wy);
        }
        const double margin = std::max(grid->getResolution(), 0.0);
        layer_->clearArea(fp_min_x - margin, fp_min_y - margin,
                          fp_max_x + margin, fp_max_y + margin);
      }
    }
  }

 private:
  std::shared_ptr<costmap_2d::SimpleObstacleLayer> layer_;
  bool footprint_clearing_{true};
  std::chrono::steady_clock::time_point last_outside_warn_at_{};
};

}  // namespace costmap
}  // namespace ayanami
