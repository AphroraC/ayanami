#include "costmap_obstacle_layer_plugin.hpp"

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

#include <costmap_2d/layered_costmap.hpp>

#include <ayanami/utility/extension_module.hpp>

namespace ayanami {
namespace modules {

// Out-of-line key function (see the header note): pins the class vtable.

CostmapObstacleLayerPlugin::~CostmapObstacleLayerPlugin() = default;

void CostmapObstacleLayerPlugin::configureParameters(
    const ayanami::ModuleContext& context) {
  const Config& file = context.config;
  const std::string& section = context.name;

  param_.combination =
      file.param<std::string>(section, "combination", param_.combination);
  param_.decay_time =
      file.param<double>(section, "decay_time", param_.decay_time);
  impl_->setDecayTime(param_.decay_time);
  param_.decay_sweep_interval = file.param<double>(
      section, "decay_sweep_interval", param_.decay_sweep_interval);
  impl_->setDecaySweepInterval(param_.decay_sweep_interval);
  param_.footprint_clearing = file.param<bool>(section, "footprint_clearing",
                                               param_.footprint_clearing);
  if (param_.combination == "overwrite") {
    impl_->setCombinationMethod(
        costmap_2d_layers::SimpleObstacleLayer::CombinationMethod::OVERWRITE);
  } else if (param_.combination == "none") {
    impl_->setCombinationMethod(
        costmap_2d_layers::SimpleObstacleLayer::CombinationMethod::NONE);
  } else {
    impl_->setCombinationMethod(
        costmap_2d_layers::SimpleObstacleLayer::CombinationMethod::MAXIMUM);
  }

  // Observation sources: names + per-source child blocks under this
  // layer's own section (same schema the server used to own).
  sources_.clear();
  for (const auto& name : file.paramStringOrList(section,
                                                 "observation_sources")) {
    Config source_cfg = file.child(section, name);
    if (!source_cfg.hasSection(name)) {
      SPDLOG_LOGGER_WARN(
        ayanami::costmapLogger(),
        "obstacle layer '{}': observation_sources declares '{}' but there "
        "is no '{}' parameter block; source ignored",
        section,
        name,
        name);
      continue;
    }
    ayanami::costmap::ObservationSourceConfig source;
    source.name = name;
    const std::string type =
        source_cfg.param<std::string>(name, "type", "laserscan");
    source.kind = type == "pointcloud"
                      ? ayanami::costmap::ObservationSourceConfig::Kind::
                            PointCloud
                      : ayanami::costmap::ObservationSourceConfig::Kind::
                            LaserScan;
    for (const auto& existing : sources_) {
      if (existing.kind == source.kind) {
        throw std::runtime_error(
            "obstacle layer '" + section + "': more than one '" + type +
            "' source ('" + existing.name + "', '" + name +
            "') is not supported; one source per sensor kind");
      }
    }
    source.marking = source_cfg.param<bool>(name, "marking", true);
    source.clearing = source_cfg.param<bool>(name, "clearing", true);
    source.min_obstacle_height =
        source_cfg.param<double>(name, "min_obstacle_height", 0.0);
    source.max_obstacle_height =
        source_cfg.param<double>(name, "max_obstacle_height", 1.0);
    source.obstacle_min_range =
        source_cfg.param<double>(name, "obstacle_min_range", 0.0);
    source.obstacle_max_range =
        source_cfg.param<double>(name, "obstacle_max_range", 3.0);
    source.raytrace_min_range =
        source_cfg.param<double>(name, "raytrace_min_range", 0.0);
    source.raytrace_max_range =
        source_cfg.param<double>(name, "raytrace_max_range", 3.5);
    source.expected_update_rate =
        source_cfg.param<double>(name, "expected_update_rate", 0.0);
    auditUnusedParameters(source_cfg, name,
                          "obstacle layer '" + section + "' source '" + name +
                              "'");
    sources_.push_back(source);
  }
}

void CostmapObstacleLayerPlugin::observe(
    const ayanami::costmap::MapObservation& observation) {
  const auto* source = observation.source;
  auto* master = impl_->layeredCostmap();
  if (source == nullptr || master == nullptr || observation.points == nullptr ||
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
  std::vector<Eigen::Vector2d> world;
  world.reserve(observation.points->size());
  if (source->marking) {
    for (const auto& p : *observation.points) {
      const Eigen::Vector3d w = t_frame_sensor * p;
      if (w.z() < source->min_obstacle_height ||
          w.z() > source->max_obstacle_height) {
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
      impl_->raytraceFreespace(*grid, sensor_origin.x(), sensor_origin.y(),
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
      impl_->markObstacles(*grid, w.x(), w.y());
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
          impl_->getName(),
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
  if (param_.footprint_clearing && observation.has_robot_pose) {
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
      impl_->clearArea(fp_min_x - margin, fp_min_y - margin,
                       fp_max_x + margin, fp_max_y + margin);
    }
  }
}

}  // namespace modules
}  // namespace ayanami

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::CostmapObstacleLayerPlugin();
}
