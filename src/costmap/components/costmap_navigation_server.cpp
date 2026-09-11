// CostmapNavigationServer main implementation.
//
// Lifecycle discipline (/): the base NavigationServerBase destructor first
// stops -> resets actions -> clearModules.
//
// Obstacle observation discipline: no global NavigationCallbacks input slot
// is registered anymore (output/observation layering); the host
// (CostmapNavigation::setPointcloud/setLaserScan) hands observations
// straight to handleObstacle*.
//
// InflationLayer::updateCosts only applies `cost > old_cost` overwrites, so
// it never de-inflates automatically once an obstacle disappears. Before
// each dynamic update, handleObstacle* first clearAreas the previous frame
// + current frame observation window (expanded by the inflation radius),
// clearing old obstacle marks and old inflation bands to FREE in the master
// grid, then markObstacles + updateMap lets the InflationLayer fully
// recompute within its recompute window, leaving no stale bands.

#include "ayanami/costmap/components/costmap_map_family_factory.hpp"
#include <ayanami/utility/bootstrap.hpp>
#include "ayanami/costmap/components/costmap_navigation_server.hpp"

#include <costmap_2d_layers/inflation_layer.hpp>
#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_2d_layers/static_layer.hpp>
#include <spdlog/spdlog.h>

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/costmap/components/costmap_commands.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/utility/config.hpp"

#include <algorithm>
#include <cassert>
#include <deque>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <mutex>
#include <shared_mutex>
#include <utility>

namespace ayanami {
namespace costmap {

namespace {
constexpr double k_default_resolution = 0.05;
constexpr double k_default_inflation_radius = 0.6;
constexpr double k_default_inscribed_radius = 0.5;
constexpr double k_default_cost_scaling_factor = 10.0;
constexpr int k_default_size_x = 0;
constexpr int k_default_size_y = 0;

// Dynamic transforms resolve on demand through the injected tf2 source
// (50 Hz). Obstacle stamping prefers querying the sensor transform with the
// scan's own timestamp (zero tolerance, no waiting):
// it places the point cloud at the "capture instant" pose -- when the robot
// rotates, a latest query would bake the omega x (now - T_scan) timing skew
// into the costmap (measured: at 1.2rad/s rotation, map-frame point clouds
// drifted 0.13-0.28m between adjacent frames, while scan-stamp queries only
// 0.01-0.04m). tf2 does not extrapolate:
// when the scan stamp is ahead of synchronized samples (faster-than-realtime
// simulation) or outside the pruning window, the query fails; then it falls
// back to latest (historical behavior) and counts the fallback, exposing the
// fallback rate via throttled logs. The robot pose (rolling centering /
// clear center) still uses latest -- window centering should follow the
// current pose.
// Query timeout policy: the injected tf2 source's timeout is a bounded poll
// budget (cv wait until a new sample or timeout, starving other TF readers/
// writers on the same global lock meanwhile) -- all core-side queries use
// timeout=0 (non-blocking) and take their own fallback paths when missing
// (latest fallback / last pose).
constexpr double k_tf_lookup_timeout_s = 0.0;

// Writer-side reader-starvation warning threshold: readers hold the shared
// lock for <10 ms per call; a writer acquisition that waits longer than this
// means some reader is overstaying its lock and map updates are delayed.
constexpr double k_writer_wait_warn_s = 0.1;

// Map-writer loop: one stamp+updateMap pass above this duration warns
// (throttled). The latest-wins observation mailbox absorbs overruns.
constexpr double k_map_update_warn_ms = 50.0;

// TF chain missing warn throttling: for the same chain (local/global), at
// most one "no transform" per k_tf_warn_interval_s, avoiding flooding at
// 5.5Hz laser per frame. Returns true when this call should warn.
constexpr double k_tf_warn_interval_s = 5.0;
constexpr double k_robot_diag_interval_s = 60.0;
bool tfWarnDue(std::atomic<std::chrono::steady_clock::time_point>& last) {
  const auto now = std::chrono::steady_clock::now();
  auto prev = last.load();
  if (now - prev < std::chrono::duration<double>(k_tf_warn_interval_s)) {
    return false;
  }
  return last.compare_exchange_strong(prev, now);
}

// Robot pose/rolling diagnostics throttling (one per 60s, for debugging; hints at a missing TF chain on failure).
bool robotDiagDue(std::atomic<std::chrono::steady_clock::time_point>& last) {
  const auto now = std::chrono::steady_clock::now();
  auto prev = last.load();
  if (now - prev < std::chrono::duration<double>(k_robot_diag_interval_s)) {
    return false;
  }
  return last.compare_exchange_strong(prev, now);
}

// Command port implementation: hands typed write commands to the sole
// writer (this server) for execution. Recovery modules only hold the
// CostmapCommandPort interface and never touch the mutable wrapper/layer.
class CommandPortImpl : public CostmapCommandPort {
 public:
  explicit CommandPortImpl(
      std::function<bool(const ClearCostmapCommand&)> handler)
      : handler(std::move(handler)) {}
  bool submit(const ClearCostmapCommand& command) override {
    return handler ? handler(command) : false;
  }

 private:
  std::function<bool(const ClearCostmapCommand&)> handler;
};
}  // namespace

CostmapNavigationServer::CostmapNavigationServer() {
  command_port = std::make_shared<CommandPortImpl>(
      [this](const ClearCostmapCommand& command) {
        // The clear command joins the writer thread's queue (single literal
        // writer); the submitter waits bounded for the applied result. With
        // no writer thread running (tests) the command executes inline.
        if (!writer_running.load()) {
          return executeClearCommand(command);
        }
        auto done = std::make_shared<std::promise<bool>>();
        auto applied = done->get_future();
        {
          std::lock_guard<std::mutex> lock(writer_mutex);
          pending_clears.push_back(PendingClear{command, std::move(done)});
        }
        writer_cv.notify_all();
        return applied.wait_for(std::chrono::seconds(2)) ==
                       std::future_status::ready
                   ? applied.get()
                   : false;
      });
}

CostmapNavigationServer::~CostmapNavigationServer() { stopWriter(); }

// ---- lifecycle: the resident map-writer thread ----

void CostmapNavigationServer::start() {
  NavigationServerBase::start();
  startWriter();
}

void CostmapNavigationServer::stop() {
  stopWriter();
  NavigationServerBase::stop();
}

void CostmapNavigationServer::startWriter() {
  if (writer_running.exchange(true)) {
    return;
  }
  // Liveness baselines start from now: a source configured with
  // expected_update_rate that never delivers goes stale after its rate.
  const auto now = std::chrono::steady_clock::now();
  source_last_seen.assign(observation_sources.size(), now);
  source_stale_warn_at.assign(observation_sources.size(),
                              std::chrono::steady_clock::time_point{});
  writer_thread = std::thread([this] { writerLoop(); });
}

void CostmapNavigationServer::stopWriter() {
  if (!writer_running.exchange(false)) {
    return;
  }
  writer_cv.notify_all();
  if (writer_thread.joinable()) {
    writer_thread.join();
  }
}

bool CostmapNavigationServer::waitForMapIdle(double timeout_s) const {
  if (!writer_running.load()) {
    return true;  // inline processing: nothing pending by construction
  }
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    if (observation_submitted.load() == observation_applied.load()) {
      std::lock_guard<std::mutex> lock(writer_mutex);
      if (pending_observations.empty()) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return observation_submitted.load() == observation_applied.load();
}

void CostmapNavigationServer::writerLoop() {
  while (writer_running.load()) {
    std::deque<PendingObservation> batch;
    std::vector<PendingClear> clears;
    {
      std::unique_lock<std::mutex> lock(writer_mutex);
      writer_cv.wait_for(lock, std::chrono::milliseconds(10), [&] {
        return !pending_observations.empty() || !pending_clears.empty() ||
               !writer_running.load();
      });
      if (!writer_running.load()) {
        break;
      }
      // Drain the whole FIFO: each queued frame is processed with its OWN
      // stamp-time TF, so no sector coverage is lost to a processing stall.
      batch.swap(pending_observations);
      clears.swap(pending_clears);
    }

    // Clear commands first: a recovery's replan depends on the cleared map.
    for (auto& clear : clears) {
      const bool ok = executeClearCommand(clear.command);
      if (clear.done) {
        clear.done->set_value(ok);
      }
    }

    // Rolling BEFORE stamping: the window origin must follow the robot with
    // minimal delay, and a scan's bounded stamp-time TF wait (up to
    // transform_tolerance inside processObservation) must not push the
    // window update into the next loop period.
    rollWindows();

    for (auto& observation : batch) {
      // nav2 mapUpdateLoop-style timing, split by responsibility: the
      // stamp-time TF wait is a DESIGNED bounded wait (up to
      // transform_tolerance -- a scan published ahead of the odometry chain
      // waits for it instead of degrading to latest), so only the
      // rasterization work past that wait is judged against the threshold.
      double tf_wait_ms = 0.0;
      bool stamp_fallback = false;
      const auto update_start = std::chrono::steady_clock::now();
      processObservation(observation, &tf_wait_ms, &stamp_fallback);
      const double update_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - update_start)
              .count();
      const double work_ms = update_ms - tf_wait_ms;
      if (work_ms > k_map_update_warn_ms) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_map_update_warn_at >= std::chrono::seconds(5)) {
          last_map_update_warn_at = now;
          spdlog::warn(
              "CostmapNavigationServer: map update work took {:.1f} ms "
              "(threshold {:.0f} ms; total {:.1f} ms incl. {:.1f} ms "
              "stamp-time TF wait); rasterization/inflation is the "
              "bottleneck -- observation ingestion stays O(enqueue)",
              work_ms, k_map_update_warn_ms, update_ms, tf_wait_ms);
        }
      }
    }
    observation_applied.store(observation_submitted.load());

    // Sensor-liveness checks live on the writer thread too (single literal
    // writer for every map mutation).
    checkSourceLiveness();
  }
}

void CostmapNavigationServer::checkSourceLiveness() {
  if (!costmap_context || !costmap_context->localCostmap()) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  bool all_fresh = true;
  for (std::size_t i = 0; i < observation_sources.size(); ++i) {
    const auto& source = observation_sources[i];
    if (source.expected_update_rate <= 0.0) {
      continue;
    }
    const double silent_s = std::chrono::duration<double>(
                                now - source_last_seen[i])
                                .count();
    if (silent_s <= source.expected_update_rate) {
      continue;
    }
    all_fresh = false;
    const auto prev = source_stale_warn_at[i];
    if (now - prev >= std::chrono::seconds(5) ||
        prev == std::chrono::steady_clock::time_point{}) {
      source_stale_warn_at[i] = now;
      spdlog::warn(
          "CostmapNavigationServer: observation source '{}' has been silent "
          "for {:.1f}s (expected every {:.1f}s); the costmap is marked "
          "not-current so the controller/execution safety checks degrade to "
          "recovery instead of trusting stale obstacle data",
          source.name, silent_s, source.expected_update_rate);
    }
  }
  if (!all_fresh) {
    costmap_context->localCostmap()->setCurrent(false);
    // A global dynamic layer consuming sensor streams degrades together
    // with the local one (same observations, same staleness).
    if (global_obstacle_layer != nullptr && costmap_context->globalCostmap()) {
      costmap_context->globalCostmap()->setCurrent(false);
    }
  }
}

void CostmapNavigationServer::configureCoreMaps(
    const std::string& config_path, ayanami::map::MapFamilyFactory::Ptr factory) {
  // The context's sole creation point = the map-family factory (falls back
  // to the costmap factory by default; core and host only know the
  // MapFamilyFactory abstract surface, the CostmapMapContext family surface
  // exists only inside this server).
  map_family_factory = factory ? std::move(factory)
                               : std::make_shared<CostmapMapFamilyFactory>();
  // Double configureCoreMaps guard. When a built context already exists,
  // re-entry is ignored: rebuilding would replace costmap_context and leave
  // the wrapper references of already-injected modules dangling (after the
  // old context is replaced by the new object, modules still point at the
  // old wrapper).
  if (costmap_context &&
      (costmap_context->globalCostmap() || costmap_context->localCostmap())) {
    spdlog::warn(
        "configureCoreMaps called twice, ignoring; existing modules would be "
        "orphaned by context replace");
    return;
  }

  ayanami::bootstrap(config_path);
  // The costmap params were merged into the config_navigation file (the original costmap.json was removed).
  Config nav_cfg(GlobalConfig::getConfigPath("config_navigation"));
  const Config& cfg = nav_cfg;

  // Frame model (nav2 convention): the global costmap lives in the
  // global_costmap_frame (typically "map"), the local rolling costmap in the
  // local_costmap_frame (typically "odom") so dynamic marks stay immune to
  // localization jumps. map_frame is still honored as a fallback alias for
  // the global frame (legacy configs).
  const std::string global_frame = cfg.param<std::string>(
      "costmap", "global_costmap_frame",
      cfg.param<std::string>(
          "costmap", "global_frame",
          cfg.param<std::string>("costmap", "map_frame", "map")));
  const std::string local_frame = cfg.param<std::string>(
      "costmap", "local_costmap_frame",
      cfg.param<std::string>("costmap", "local_frame", "odom"));
  // robot_frame single source of truth = config_navigation's
  // navigation::robot_frame.
  const std::string robot_frame =
      nav_cfg.param<std::string>("navigation", "robot_frame", "base_link");
  const double resolution =
      cfg.param<double>("costmap", "resolution", k_default_resolution);

  // ---- global: StaticLayer + InflationLayer (static content only) ----
  // Map-representation policy (nav2 convention): the global costmap carries
  // ONLY static + inflation content. Dynamic obstacle marks live exclusively
  // in the local (odom-frame) rolling map, so no global dynamic layer can go
  // stale under localization corrections and no drift-heuristic clearing is
  // needed.
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      global_frame,
      cfg.param<bool>("costmap_global", "rolling_window", false),
      cfg.param<bool>("costmap_global", "track_unknown", false));
  // Layer plugin path: a "layers" list in the map section loads the
  // composition from module jsons (list order = composition order, the
  // modules/ so+json pattern); a failed load in the plugin path is a
  // startup error. Absent -> the legacy hardcoded composition below.
  const bool global_layers_loaded =
      loadLayerPlugins("costmap_global", cfg, config_path, global_lm,
                       global_layer_modules, global_layer_plugins);
  // Function scope: the legacy writer's pointer registration below reads
  // it even when the plugin path skipped the legacy construction.
  std::shared_ptr<costmap_2d_layers::SimpleObstacleLayer> global_obstacle;
  std::shared_ptr<costmap_2d_layers::SimpleObstacleLayer> obstacle_layer;
  if (!global_layers_loaded) {
    auto static_layer = std::make_shared<costmap_2d_layers::StaticLayer>();
    global_lm->addLayer(static_layer);
    const std::string static_map =
        cfg.param<std::string>("costmap_global", "static_map", "");
    if (!static_map.empty()) {
      try {
        // A relative static_map (assets/nav.pgm) resolves relative to the
        // package root (the parent of config/), no longer depending on the
        // node cwd (~/.ros under roslaunch).
        std::string map_path = static_map;
        if (map_path.front() != '/' && map_path.find("://") == std::string::npos) {
          map_path = std::filesystem::path(config_path).parent_path().string() +
                     "/" + static_map;
        }
        // Ayanami map metadata is JSON-only and loaded through the common Config
        // API. Middleware adapters keep their deployment metadata outside this package.
        std::filesystem::path metadata_path(map_path);
        metadata_path.replace_extension(".json");
        Config map_cfg(metadata_path.string());
        const auto map_resolution = map_cfg.param<double>("map", "resolution");
        const auto map_origin = map_cfg.param<Eigen::Vector3d>("map", "origin");
        if (!map_resolution || !map_origin) {
          throw std::runtime_error("invalid JSON map metadata: " +
                                   metadata_path.string());
        }
        static_layer->setMapPath(map_path, *map_resolution, map_origin->x(),
                                 map_origin->y());
      } catch (const std::exception& e) {
        spdlog::error("CostmapNavigationServer::configureCoreMaps: {}", e.what());
      }
    }
    if (!static_layer->hasMap()) {
      const int fallback_size_x =
          cfg.param<int>("costmap_global", "size_x", k_default_size_x);
      const int fallback_size_y =
          cfg.param<int>("costmap_global", "size_y", k_default_size_y);
      if (fallback_size_x > 0 && fallback_size_y > 0) {
        global_lm->resizeMap(static_cast<unsigned int>(fallback_size_x),
                             static_cast<unsigned int>(fallback_size_y),
                             resolution, 0.0, 0.0);
      } else {
        throw std::runtime_error(
            "static global map failed to load and no fallback size is configured");
      }
    }
    {
      const auto* global_master = global_lm->getCostmap();
      if (!global_master || global_master->getSizeInCellsX() == 0 ||
          global_master->getSizeInCellsY() == 0) {
        throw std::runtime_error("static map loaded but global master geometry is empty");
      }
      spdlog::info(
          "global costmap geometry from static map: {}x{} resolution={} "
          "origin=({}, {})",
          global_master->getSizeInCellsX(), global_master->getSizeInCellsY(),
          global_master->getResolution(), global_master->getOriginX(),
          global_master->getOriginY());
    }

    // ---- global dynamic obstacle layer (nav2 global_costmap shape) ----
    // costmap_global.observation_sources names the sources (blocks under
    // costmap_local) whose observations are ALSO stamped into the global map;
    // empty (default) keeps a static-only global map. The layer sits between
    // the StaticLayer and the InflationLayer: marks inflate like real
    // obstacles, and the StaticLayer below restores the base map on every
    // dirty-window update (clear-to-free stays off so clears only retire
    // dynamic marks).
    global_observation_sources =
        cfg.paramStringOrList("costmap_global", "observation_sources");
    if (!global_observation_sources.empty()) {
      global_obstacle =
          std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
      global_obstacle->setCombinationMethod(
          costmap_2d_layers::SimpleObstacleLayer::CombinationMethod::OVERWRITE);
      global_obstacle->setClearToFree(false);
      global_lm->addLayer(global_obstacle);
    }

    auto global_inflation = std::make_shared<costmap_2d_layers::InflationLayer>();
    const double global_inflation_radius = cfg.param<double>(
        "costmap_global", "inflation_radius", k_default_inflation_radius);
    global_inflation->setInflationRadius(global_inflation_radius);
    this->global_inflation_radius = global_inflation_radius;
    global_inflation->setInscribedRadius(cfg.param<double>(
        "costmap_global", "inscribed_radius", k_default_inscribed_radius));
    global_inflation->setCostScalingFactor(cfg.param<double>(
        "costmap_global", "cost_scaling_factor", k_default_cost_scaling_factor));
    global_lm->addLayer(global_inflation);
  }
  for (const auto& plugin : global_layer_plugins) {
    plugin->onAddedToMap(*global_lm);
  }
  // ---- local: SimpleObstacleLayer + InflationLayer (rolling, LOCAL frame) ----
  auto local_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      local_frame,
      cfg.param<bool>("costmap_local", "rolling_window", true),
      cfg.param<bool>("costmap_local", "track_unknown", false));
  const int local_size_x =
      cfg.param<int>("costmap_local", "size_x", k_default_size_x);
  const int local_size_y =
      cfg.param<int>("costmap_local", "size_y", k_default_size_y);
  if (local_size_x > 0 && local_size_y > 0) {
    local_lm->resizeMap(static_cast<unsigned int>(local_size_x),
                        static_cast<unsigned int>(local_size_y), resolution,
                        0.0, 0.0);
  }

  // Layer plugin path for the local map (see the global note above); the
  // map geometry (size/resolution/rolling) stays a server-level concern.
  const double local_inflation_radius = cfg.param<double>(
      "costmap_local", "inflation_radius", k_default_inflation_radius);
  const bool local_layers_loaded =
      loadLayerPlugins("costmap_local", cfg, config_path, local_lm,
                       local_layer_modules, local_layer_plugins);
  if (!local_layers_loaded) {
    obstacle_layer = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
    obstacle_layer->setCombinationMethod(
        costmap_2d_layers::SimpleObstacleLayer::CombinationMethod::OVERWRITE);
    local_lm->addLayer(obstacle_layer);

    auto local_inflation = std::make_shared<costmap_2d_layers::InflationLayer>();
    local_inflation->setInflationRadius(local_inflation_radius);
    local_inflation->setInscribedRadius(cfg.param<double>(
        "costmap_local", "inscribed_radius", k_default_inscribed_radius));
    local_inflation->setCostScalingFactor(cfg.param<double>(
        "costmap_local", "cost_scaling_factor", k_default_cost_scaling_factor));
    local_lm->addLayer(local_inflation);
  }
  auto local_wrapper =
      std::make_shared<costmap_2d::CostmapWrapper>(local_lm, local_frame);
  // Materialize the local map once at configure: layers start non-current,
  // and until the first updateMap the execution-layer safetyCheck would
  // gate the controller/recovery loops on a not-yet-current map.
  local_wrapper->updateMap(0.0, 0.0, 0.0);

  // Robot footprint sources (nav_demo parameter semantics):
  // - costmap::footprint polygon (nav2 semantics, highest priority)
  // - costmap::robot_radius > 0 -> generate a 16-gon circle approximation
  //   (ROS1 robot_radius semantics)
  // - both absent -> not set (the inscribed_radius parameter serves as the
  //   inflation layer fallback)
  costmap_2d::Footprint footprint;
  const auto footprint_values = cfg.param<std::vector<Eigen::Vector2d>>(
      "costmap", "footprint", {});
  footprint.reserve(footprint_values.size());
  for (const auto& point : footprint_values) {
    footprint.push_back(costmap_2d::Point{point.x(), point.y(), 0.0});
  }
  if (footprint.empty()) {
    const double robot_radius =
        cfg.param<double>("costmap", "robot_radius", 0.0);
    if (robot_radius > 0.0) {
      constexpr int k_circle_segments = 16;
      footprint.reserve(k_circle_segments);
      for (int i = 0; i < k_circle_segments; ++i) {
        const double angle = 2.0 * M_PI * static_cast<double>(i) /
                             static_cast<double>(k_circle_segments);
        footprint.push_back(costmap_2d::Point{
            robot_radius * std::cos(angle), robot_radius * std::sin(angle),
            0.0});
      }
    }
  }
  if (!footprint.empty()) {
    // Nav2 semantics: a configured footprint is the source of truth for the
    // inscribed radius. The explicit radius above is only a fallback when no
    // polygon is configured.
    global_lm->setFootprint(footprint);
    local_lm->setFootprint(footprint);
  }

  auto global_wrapper =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, global_frame);
  // Materialize once after footprint propagation, avoiding a second full-map
  // reinflation during startup.
  global_wrapper->updateMap(0.0, 0.0, 0.0);

  local_obstacle_layer = obstacle_layer.get();
  global_obstacle_layer = global_obstacle.get();
  this->local_inflation_radius = local_inflation_radius;
  local_resolution = resolution;
 // Flat max-range keys stay as back-compat defaults for the per-source
 // ranges (nav2 names them obstacle_max_range / raytrace_max_range).
  const double local_obstacle_range =
      cfg.param<double>("costmap_local", "obstacle_range", 3.0);
  const double local_raytrace_range =
      cfg.param<double>("costmap_local", "raytrace_range", 3.5);
 // Decoupled rolling-window update frequency (periodic rolling driven by the
 // map-writer thread, independent of the sensor stream)
  const double local_update_hz =
      cfg.param<double>("costmap_local", "update_frequency", 10.0);
  const double global_update_hz =
      cfg.param<double>("costmap_global", "update_frequency", 5.0);
  local_roll_period = local_update_hz > 0.0
                          ? std::chrono::duration<double>(1.0 / local_update_hz)
                          : std::chrono::duration<double>(0.0);
  global_roll_period = global_update_hz > 0.0
                           ? std::chrono::duration<double>(1.0 / global_update_hz)
                           : std::chrono::duration<double>(0.0);

  // ---- observation_sources as a real contract (nav2 semantics) ----
  // Each declared source name reads its own child block
  // (costmap_local.<name>.{type, marking, clearing, min/max_obstacle_height,
  // obstacle_min/max_range, raytrace_min/max_range, expected_update_rate});
  // every declared block participates in the unused-keys audit.
  observation_sources.clear();
  for (const auto& name :
       cfg.paramStringOrList("costmap_local", "observation_sources")) {
    Config source_cfg = cfg.child("costmap_local", name);
    if (!source_cfg.hasSection(name)) {
      spdlog::warn(
          "costmap_local.observation_sources declares '{}' but there is no "
          "'{}' parameter block under costmap_local; source ignored",
          name, name);
      continue;
    }
    ObservationSourceConfig source;
    source.name = name;
    const std::string type =
        source_cfg.param<std::string>(name, "type", "laserscan");
    source.kind = type == "pointcloud"
                      ? ObservationSourceConfig::Kind::PointCloud
                      : ObservationSourceConfig::Kind::LaserScan;
    for (const auto& existing : observation_sources) {
      if (existing.kind == source.kind) {
        throw std::runtime_error(
            "costmap_local.observation_sources: more than one '" + type +
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
    source.obstacle_max_range = source_cfg.param<double>(
        name, "obstacle_max_range", local_obstacle_range);
    source.raytrace_min_range =
        source_cfg.param<double>(name, "raytrace_min_range", 0.0);
    source.raytrace_max_range = source_cfg.param<double>(
        name, "raytrace_max_range", local_raytrace_range);
    source.expected_update_rate =
        source_cfg.param<double>(name, "expected_update_rate", 0.0);
    auditUnusedParameters(source_cfg, name,
                          "observation source '" + name + "'");
    observation_sources.push_back(source);
  }
  // Global source names must reference declared source blocks; an unknown
  // name would otherwise silently consume nothing.
  for (const auto& name : global_observation_sources) {
    const bool known =
        std::find_if(observation_sources.begin(), observation_sources.end(),
                     [&name](const ObservationSourceConfig& s) {
                       return s.name == name;
                     }) != observation_sources.end();
    if (!known) {
      spdlog::warn(
          "costmap_global.observation_sources declares '{}' but there is no "
          "'{}' parameter block under costmap_local; the global map ignores "
          "it",
          name, name);
    }
  }
  // Union of the legacy source blocks + every layer plugin's declared
  // sources: the writer's kind lookup and the liveness check see one list
  // (dedup by name; a name declared twice keeps its first definition).
  for (const auto* map_plugins :
       {&global_layer_plugins, &local_layer_plugins}) {
    for (const auto& plugin : *map_plugins) {
      for (const auto& plugin_source : plugin->observationSources()) {
        const bool known =
            std::find_if(observation_sources.begin(),
                         observation_sources.end(),
                         [&plugin_source](const ObservationSourceConfig& x) {
                           return x.name == plugin_source.name;
                         }) != observation_sources.end();
        if (!known) {
          observation_sources.push_back(plugin_source);
        }
      }
    }
  }
  // Liveness baselines: the writer start re-baselines them, but the inline
  // (no-writer) processing path stamps them too, so size them here.
  source_last_seen.assign(observation_sources.size(),
                          std::chrono::steady_clock::now());
  source_stale_warn_at.assign(observation_sources.size(),
                              std::chrono::steady_clock::time_point{});
  // Stamp-time TF bounded wait (writer-thread budget; publishers never
  // block) + footprint clearing (the robot never marks itself).
  transform_tolerance =
      cfg.param<double>("costmap_local", "transform_tolerance", 0.1);
  footprint_clearing =
      cfg.param<bool>("costmap_local", "footprint_clearing", true);

  costmap_context =
      std::dynamic_pointer_cast<CostmapMapContext>(map_family_factory->createContext());
  if (!costmap_context) {
    throw std::runtime_error(
        "map family factory '" + map_family_factory->familyName() +
        "' did not create a CostmapMapContext");
  }
  costmap_context->setGlobalCostmap(global_wrapper);
  costmap_context->setLocalCostmap(local_wrapper);
  costmap_context->setFrames(global_frame, local_frame, robot_frame);
  // Writer-side reader-starvation observability (see CostmapWrapper).
  global_wrapper->setWriterWaitWarn(k_writer_wait_warn_s);
  local_wrapper->setWriterWaitWarn(k_writer_wait_warn_s);
  // The footprint is cached as an immutable copy: snapshot building reads
  // only this cache (never read the live LayeredCostmap's footprint outside
  // the write lock).
  costmap_context->setFootprint(footprint);
  // The map write command port is exposed via the context
  // (AbstractMapContext::commandPort):
  // family modules such as recovery fetch it from the map context at
  // configure time instead of side-path injection.
  costmap_context->setCommandPort(command_port);

  // One-time configuration of the global RobotStateController: frame names
  // + velocity timeout are cached only here; the hot getRobotState never
  // reads Config (extension).
  const double velocity_timeout =
      nav_cfg.param<double>("navigation", "velocity_timeout", 0.25);
  ayanami::components::RobotStateController::instance().initialize(
      global_frame, local_frame, robot_frame, velocity_timeout);

  prev_local = {};
  prev_global = {};
  has_last_local_robot = false;
}

bool CostmapNavigationServer::loadLayerPlugins(
    const std::string& section, const ayanami::Config& cfg,
    const std::string& config_dir,
    const costmap_2d::LayeredCostmap::Ptr& layered_costmap,
    ayanami::components::ModuleManager<ayanami::costmap::CostmapLayerPlugin>&
        manager,
    std::vector<ayanami::costmap::CostmapLayerPlugin::Ptr>& plugins) {
  const auto files = cfg.paramStringOrList(section, "layers");
  if (files.empty()) {
    return false;  // no "layers" list -> legacy hardcoded composition
  }
  for (const auto& file : files) {
    if (file.empty()) {
      continue;
    }
    // Module json paths resolve relative to the config directory (the
    // config_recovery convention).
    const std::string path = std::filesystem::path(file).is_absolute()
                                 ? file
                                 : (std::filesystem::path(config_dir) / file)
                                       .string();
    Config layer_cfg(path);
    ayanami::components::ModuleSpec spec;
    spec.name = layer_cfg.param<std::string>("layer", "name", "");
    spec.so_name = layer_cfg.param<std::string>("layer", "so_name", "");
    if (!manager.loadModule(spec, ayanami::ModuleContext{spec.name, layer_cfg},
                            {})) {
      throw std::runtime_error(
          "layer plugin failed to load: " + spec.name + " (" + spec.so_name +
          "): " + manager.lastFailureMessage());
    }
    auto plugin = manager.getModule(spec.name);
    layered_costmap->addLayer(plugin->layer());
    plugins.push_back(plugin);
  }
  return true;
}

// ---- Virtual initialize*Module hooks: costmap family injection ----

bool CostmapNavigationServer::initializePlannerModule(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractPlanner::Ptr ptr) {
  if (!costmap_context) {
    spdlog::error(
        "CostmapNavigationServer::initializePlannerModule('{}'): costmap "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  auto cast = std::dynamic_pointer_cast<CostmapPlanner>(ptr);
  if (!cast) {
    spdlog::error(
        "CostmapNavigationServer::initializePlannerModule('{}'): module is "
        "not a CostmapPlanner (costmap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::ModuleContext{name, module_config},
                  costmap_context->getSharedConstPtr());
  return true;
}

bool CostmapNavigationServer::initializeSmootherModule(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractSmoother::Ptr ptr) {
  if (!costmap_context) {
    spdlog::error(
        "CostmapNavigationServer::initializeSmootherModule('{}'): costmap "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  auto cast = std::dynamic_pointer_cast<CostmapSmoother>(ptr);
  if (!cast) {
    spdlog::error(
        "CostmapNavigationServer::initializeSmootherModule('{}'): module is "
        "not a CostmapSmoother (costmap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::ModuleContext{name, module_config},
                  costmap_context->getSharedConstPtr());
  return true;
}

bool CostmapNavigationServer::initializeControllerModule(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractController::Ptr ptr) {
  if (!costmap_context) {
    spdlog::error(
        "CostmapNavigationServer::initializeControllerModule('{}'): costmap "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  // Strict costmap family constraint: only CostmapController is accepted;
  // plain AbstractControllers are no longer allowed (costmap_pid_controller now derives
  // from CostmapController).
  auto cast = std::dynamic_pointer_cast<CostmapController>(ptr);
  if (!cast) {
    spdlog::error(
        "CostmapNavigationServer::initializeControllerModule('{}'): module is "
        "not a CostmapController (costmap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::ModuleContext{name, module_config},
                  costmap_context->getSharedConstPtr());
  return true;
}

bool CostmapNavigationServer::initializeRecoveryModule(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractRecovery::Ptr ptr) {
  if (!costmap_context) {
    spdlog::error(
        "CostmapNavigationServer::initializeRecoveryModule('{}'): costmap "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  auto cast = std::dynamic_pointer_cast<CostmapRecovery>(ptr);
  if (!cast) {
    spdlog::error(
        "CostmapNavigationServer::initializeRecoveryModule('{}'): module is "
        "not a CostmapRecovery (costmap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::ModuleContext{name, module_config},
                  costmap_context->getSharedConstPtr());
  // Stash the module config so createActions can inject per-module
  // execution settings (tick cycle period).
  recovery_configs.erase(name);
  recovery_configs.emplace(name, module_config);
  // Map write command port: recovery fetches it via map->commandPort
  // inside configure (CostmapCommandPort), executed by this server (the
  // sole writer).
  return true;
}

// ---- createActions: production path = Costmap*Action ----

void CostmapNavigationServer::createActions() {
  planner_actions.clear();
  smoother_actions.clear();
  controller_actions.clear();
  recovery_actions.clear();

  // planner/smoother/controller: one Costmap*Action per loaded module
  // (Costmap*Execution carrying the ctx; nav2 multi-module semantics:
  // requests select by goal id, empty id = default = first configured).
  for (const auto& name : planner_modules.getLoadedNames()) {
    auto planner = planner_modules.getModule(name);
    if (!planner) {
      continue;
    }
    planner_actions[name] = std::make_unique<CostmapPlannerAction>(
        planner, costmap_context);
  }
  if (!planner_actions.empty() &&
      planner_actions.find(default_planner) == planner_actions.end()) {
    spdlog::error("CostmapNavigationServer::createActions: default planner "
                  "'{}' not loaded",
                  default_planner);
  }

  for (const auto& name : smoother_modules.getLoadedNames()) {
    auto smoother = smoother_modules.getModule(name);
    if (!smoother) {
      continue;
    }
    smoother_actions[name] = std::make_unique<CostmapSmootherAction>(
        smoother, costmap_context);
  }
  if (!smoother_actions.empty() &&
      smoother_actions.find(default_smoother) == smoother_actions.end()) {
    spdlog::error("CostmapNavigationServer::createActions: default smoother "
                  "'{}' not loaded",
                  default_smoother);
  }

  // controller: CostmapControllerExecution + checker/velocity-limit/real
  // pose-source assembly.
  for (const auto& name : controller_modules.getLoadedNames()) {
    auto controller = controller_modules.getModule(name);
    if (!controller) {
      continue;
    }
    controller_actions[name] = std::make_unique<CostmapControllerAction>(
        controller, costmap_context,
        [this](ayanami::components::ControllerExecution::Ptr exec) {
          configureControllerExecution(exec);
        });
  }
  if (!controller_actions.empty() &&
      controller_actions.find(default_controller) == controller_actions.end()) {
    spdlog::error("CostmapNavigationServer::createActions: default controller "
                  "'{}' not loaded",
                  default_controller);
  }

  // recovery: CostmapRecoveryExecution + velocity-limit assembly +
  // per-module tick cycle period (recovery.sim_cycle_dt, seconds).
  for (const auto& name : recovery_modules.getLoadedNames()) {
    auto recovery = recovery_modules.getModule(name);
    if (!recovery) {
      continue;
    }
    double cycle_period_s = 0.01;
    const auto config_iter = recovery_configs.find(name);
    if (config_iter != recovery_configs.end()) {
      cycle_period_s = config_iter->second.param<double>(
          "recovery", "sim_cycle_dt", cycle_period_s);
    }
    auto action = std::make_unique<CostmapRecoveryAction>(
        recovery, costmap_context,
        [this, cycle_period_s](
            ayanami::components::RecoveryExecution::Ptr exec) {
          configureRecoveryExecution(exec);
          if (cycle_period_s > 0.0) {
            exec->setCyclePeriod(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::duration<double>(cycle_period_s)));
          }
        });
    recovery_actions[name] = std::move(action);
  }
}

// ---- Obstacle observation map stamping ----

void CostmapNavigationServer::handleObstaclePointCloud(
    const ayanami::sensor_msgs::PointCloud2& cloud) {
  if (!costmap_context || !costmap_context->localCostmap()) {
    spdlog::warn(
        "CostmapNavigationServer::handleObstaclePointCloud: costmap not "
        "configured (call configureCoreMaps first)");
    return;
  }
  const auto* source = findSource(ObservationSourceConfig::Kind::PointCloud);
  if (source == nullptr) {
    static std::atomic<bool> warned{false};
    if (!warned.exchange(true)) {
      spdlog::warn(
          "handleObstaclePointCloud: no pointcloud observation source is "
          "configured (costmap_local.observation_sources); cloud dropped");
    }
    return;
  }
  PendingObservation observation;
  observation.source = source;
  observation.frame_id = cloud.header.frame_id;
  observation.stamp = cloud.header.stamp;
  observation.points.reserve(cloud.cloud.points.size());
  for (const auto& p : cloud.cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    // z is KEPT here: the source's min/max_obstacle_height window filters
    // hits in processObservation (nav2 semantics).
    observation.points.emplace_back(static_cast<double>(p.x),
                                    static_cast<double>(p.y),
                                    static_cast<double>(p.z));
  }
  submitObservation(std::move(observation));
}

void CostmapNavigationServer::handleObstacleLaserScan(
    const ayanami::sensor_msgs::LaserScan& scan) {
  if (!costmap_context || !costmap_context->localCostmap()) {
    spdlog::warn(
        "CostmapNavigationServer::handleObstacleLaserScan: costmap not "
        "configured (call configureCoreMaps first)");
    return;
  }
  if (scan.ranges.empty()) {
    return;
  }
  const auto* source = findSource(ObservationSourceConfig::Kind::LaserScan);
  if (source == nullptr) {
    static std::atomic<bool> warned{false};
    if (!warned.exchange(true)) {
      spdlog::warn(
          "handleObstacleLaserScan: no laserscan observation source is "
          "configured (costmap_local.observation_sources); scan dropped");
    }
    return;
  }
  PendingObservation observation;
  observation.source = source;
  observation.frame_id = scan.header.frame_id;
  observation.stamp = scan.header.stamp;
  observation.points.reserve(scan.ranges.size());
  observation.clearing_points.reserve(scan.ranges.size());
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = static_cast<double>(scan.ranges[i]);
    const double angle = scan.angle_min + i * scan.angle_increment;
    // Ray endpoints clamped to the source's raytrace_max_range (a
    // non-finite range rays out to the max: "nothing up to here is
    // blocked"); processObservation skips rays below raytrace_min_range.
    const double clear_range =
        std::isfinite(range) ? std::clamp(range, 0.0,
                                          source->raytrace_max_range)
                             : source->raytrace_max_range;
    observation.clearing_points.emplace_back(clear_range * std::cos(angle),
                                             clear_range * std::sin(angle),
                                             0.0);
    if (range >= scan.range_min && range <= scan.range_max) {
      observation.points.emplace_back(range * std::cos(angle),
                                      range * std::sin(angle), 0.0);
    }
  }
  submitObservation(std::move(observation));
}

const ObservationSourceConfig*
CostmapNavigationServer::findSource(ObservationSourceConfig::Kind kind) const {
  for (const auto& source : observation_sources) {
    if (source.kind == kind) {
      return &source;
    }
  }
  return nullptr;
}

void CostmapNavigationServer::submitObservation(
    PendingObservation&& observation) {
  observation_submitted.fetch_add(1);
  if (!writer_running.load()) {
    // No writer thread (direct-construction tests): legacy inline semantics,
    // the caller's thread stamps the map synchronously.
    processObservation(observation);
    observation_applied.store(observation_submitted.load());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(writer_mutex);
    if (pending_observations.size() >= k_max_pending_observations) {
      // Sustained processing stall: drop the OLDEST frame (its sector
      // coverage is the stalest) and keep the freshest ones.
      pending_observations.pop_front();
    }
    pending_observations.push_back(std::move(observation));
  }
  writer_cv.notify_all();
}

bool CostmapNavigationServer::lookupWithBoundedWait(
    const std::string& target_frame, const std::string& source_frame,
    const ayanami::Timestamp& stamp, Eigen::Isometry3d& transform,
    double* wait_seconds) const {
  auto& robot_state_controller =
      ayanami::components::RobotStateController::instance();
  // Bounded POLL (never a blocking wait on the ros clock): the writer thread is
  // allowed to wait up to transform_tolerance for the stamp-time transform
  // (publishers are never blocked -- they only enqueue), and polling with
  // zero-timeout lookups avoids starving other TF readers on the store's
  // internal lock. The 5ms poll granularity halves the quantization delay a
  // just-behind-the-chain scan stamp experiences.
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(transform_tolerance);
  const auto wait_start = std::chrono::steady_clock::now();
  while (true) {
    if (robot_state_controller.lookupTransform(target_frame, source_frame,
                                               stamp, transform,
                                               k_tf_lookup_timeout_s)) {
      if (wait_seconds) {
        *wait_seconds = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - wait_start)
                            .count();
      }
      return true;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      if (wait_seconds) {
        *wait_seconds = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - wait_start)
                            .count();
      }
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void CostmapNavigationServer::clearFootprintRegion(
    costmap_2d_layers::SimpleObstacleLayer& layer, double robot_x,
    double robot_y, double robot_yaw) {
  const auto& footprint = costmap_context->footprint();
  if (footprint.empty()) {
    return;
  }
  // Rotated footprint bounding box (+ one cell margin): clearing slightly
  // beyond the polygon is free space anyway under the robot.
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  for (const auto& point : footprint) {
    const double wx = robot_x + point.x * cos_yaw - point.y * sin_yaw;
    const double wy = robot_y + point.x * sin_yaw + point.y * cos_yaw;
    min_x = std::min(min_x, wx);
    min_y = std::min(min_y, wy);
    max_x = std::max(max_x, wx);
    max_y = std::max(max_y, wy);
  }
  const double margin = std::max(local_resolution, 0.0);
  layer.clearArea(min_x - margin, min_y - margin, max_x + margin,
                  max_y + margin);
}

bool CostmapNavigationServer::stampObservationOnMap(
    const PendingObservation& observation, const std::string& map_frame,
    const costmap_2d::CostmapWrapper::Ptr& wrapper,
    costmap_2d_layers::SimpleObstacleLayer* obstacle_layer,
    PrevMapObservation& prev, double inflation_radius, bool is_local_map,
    double* tf_wait_ms, bool* stamp_fallback,
    const std::vector<ayanami::costmap::CostmapLayerPlugin::Ptr>& plugins) {
  if (tf_wait_ms) {
    *tf_wait_ms = 0.0;
  }
  if (stamp_fallback) {
    *stamp_fallback = false;
  }
  const auto* source = observation.source;
  if (wrapper == nullptr || source == nullptr || !costmap_context ||
      (obstacle_layer == nullptr && plugins.empty())) {
    return false;
  }
  auto& robot_state_controller =
      ayanami::components::RobotStateController::instance();
  const bool have_stamp = !observation.stamp.isLatest();

  // Sensor origin: stamp-time TF with a bounded in-writer wait; on final
  // failure fall back to latest and count the fallback (the fallback bakes
  // an omega x dt skew into the marks while rotating -- persistently high
  // fallback rates mean the scan stamp deviates beyond the sync window).
  Eigen::Isometry3d t_frame_sensor = Eigen::Isometry3d::Identity();
  bool stamp_ok = false;
  double lookup_wait_ms = 0.0;
  if (have_stamp &&
      lookupWithBoundedWait(map_frame, observation.frame_id,
                            observation.stamp, t_frame_sensor,
                            &lookup_wait_ms)) {
    stamp_ok = true;
    ++stamp_query_ok;
    if (tf_wait_ms) {
      *tf_wait_ms = lookup_wait_ms;
    }
  }
  if (!stamp_ok &&
      !robot_state_controller.lookupTransform(
          map_frame, observation.frame_id, ayanami::Timestamp::latest(),
          t_frame_sensor, k_tf_lookup_timeout_s)) {
    if (stamp_fallback) {
      *stamp_fallback = true;
    }
    if (have_stamp) {
      ++stamp_query_fallback;
    }
    if (tfWarnDue(map_tf_warn_at)) {
      spdlog::warn(
          "CostmapNavigationServer: no transform {} <- {} (stamp={:f}s); "
          "obstacle stamping paused. Check the TF producers (gazebo "
          "diff-drive odom->base_footprint, robot_state_publisher) and "
          "whether a leftover simulation instance publishes duplicate TF",
          map_frame, observation.frame_id, observation.stamp.toSeconds());
    }
    return false;
  }

  // Robot pose in this map's frame before the write lock. The LOCAL map
  // keeps a rolling-window pose cache: on a transient TF loss roll the
  // window with the last successful pose (never (0,0,0) -- the rolling
  // window would anchor back to the frame origin and DWB would find no
  // path once the robot leaves it). The global map needs the pose only for
  // footprint clearing; on failure that pass is skipped.
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  bool have_robot_pose = false;
  Eigen::Isometry3d t_robot = Eigen::Isometry3d::Identity();
  if (robot_state_controller.lookupTransform(
          map_frame, costmap_context->robotFrame(),
          ayanami::Timestamp::latest(), t_robot, k_tf_lookup_timeout_s)) {
    robot_x = t_robot.translation().x();
    robot_y = t_robot.translation().y();
    robot_yaw = std::atan2(t_robot.linear()(1, 0), t_robot.linear()(0, 0));
    have_robot_pose = true;
    if (is_local_map) {
      has_last_local_robot = true;
      last_local_robot_x = robot_x;
      last_local_robot_y = robot_y;
      last_local_robot_yaw = robot_yaw;
    }
  } else if (is_local_map && has_last_local_robot) {
    robot_x = last_local_robot_x;
    robot_y = last_local_robot_y;
    robot_yaw = last_local_robot_yaw;
    have_robot_pose = true;
  }

  // Source-driven hit filtering: height window (z kept through conversion)
  // and marking range window from the sensor origin (nav2 semantics). The
  // surviving points move into THIS map's frame; their bbox feeds the
  // previous-window clearing.
  const Eigen::Vector3d sensor_origin = t_frame_sensor.translation();
  std::vector<Eigen::Vector2d> world;
  world.reserve(observation.points.size());
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  if (source->marking) {
    for (const auto& p : observation.points) {
      // nav2 semantics: the height window is judged in the MAP frame --
      // points are transformed FIRST, then filtered. Filtering in the
      // sensor frame (the old behavior) lets any mounting tilt or pitch
      // silently drop whole sectors of the cloud ("dense cloud, no
      // obstacle"): sensor-frame z there says nothing about the height
      // above the ground plane.
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
      min_x = std::min(min_x, w.x());
      min_y = std::min(min_y, w.y());
      max_x = std::max(max_x, w.x());
      max_y = std::max(max_y, w.y());
    }
  }

  // Write pass (sole writer thread; each map takes only its own lock).
  auto write_lock = wrapper->lockForWrite();
  if (!plugins.empty()) {
    // Plugin path: the consuming layers own the filtering policy; the
    // writer only resolves the per-map TF/pose, routes by source kind and
    // publishes the map afterwards.
    ayanami::costmap::MapObservation map_observation;
    map_observation.t_frame_sensor = t_frame_sensor;
    map_observation.points = &observation.points;
    map_observation.clearing_points = &observation.clearing_points;
    map_observation.robot_pose_xy_yaw =
        Eigen::Vector3d(robot_x, robot_y, robot_yaw);
    map_observation.has_robot_pose = have_robot_pose;
    for (const auto& plugin : plugins) {
      for (const auto& plugin_source : plugin->observationSources()) {
        if (plugin_source.kind != source->kind) {
          continue;
        }
        map_observation.source = &plugin_source;
        plugin->observe(map_observation);
        break;
      }
    }
    wrapper->updateMapUnlocked(robot_x, robot_y, robot_yaw);
    wrapper->setCurrent(true);
    return true;
  }
  auto* grid = wrapper->get().getCostmap();
  if (source->clearing) {
    if (!observation.clearing_points.empty()) {
      for (const auto& endpoint : observation.clearing_points) {
        const double range = endpoint.head<2>().norm();
        if (range < source->raytrace_min_range) {
          continue;
        }
        Eigen::Vector3d clamped = endpoint;
        if (range > source->raytrace_max_range) {
          clamped = endpoint * (source->raytrace_max_range / range);
        }
        const Eigen::Vector3d world_end = t_frame_sensor * clamped;
        obstacle_layer->raytraceFreespace(*grid, sensor_origin.x(),
                                          sensor_origin.y(), world_end.x(),
                                          world_end.y());
      }
    } else if (prev.valid) {
      double clear_min_x = min_x;
      double clear_min_y = min_y;
      double clear_max_x = max_x;
      double clear_max_y = max_y;
      for (const auto& old : prev.points) {
        clear_min_x = std::min(clear_min_x, old.x());
        clear_min_y = std::min(clear_min_y, old.y());
        clear_max_x = std::max(clear_max_x, old.x());
        clear_max_y = std::max(clear_max_y, old.y());
      }
      // The expansion margin is the inflation radius + one cell: the band
      // can reach up to ceil(radius/res) cells, and world->cell rounding
      // would exclude cells lying exactly on the radius boundary.
      const double margin =
          inflation_radius + std::max(local_resolution, 0.0);
      obstacle_layer->clearArea(clear_min_x - margin, clear_min_y - margin,
                                clear_max_x + margin, clear_max_y + margin);
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
      obstacle_layer->markObstacles(w.x(), w.y());
    }
    if (outside_window > 0) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_outside_window_warn_at >= std::chrono::seconds(5)) {
        last_outside_window_warn_at = now;
        spdlog::warn(
            "CostmapNavigationServer: {} of {} marked points fell outside "
            "the '{}' map ({:.0f}x{:.0f} m); enlarge the map size or reduce "
            "obstacle_max_range",
            outside_window, world.size(), map_frame,
            grid->getSizeInMetersX(), grid->getSizeInMetersY());
      }
    }
  }

  // Footprint clearing LAST (nav2 semantics): hits from the robot's own
  // chassis lose to the clear, so the robot can never mark itself with
  // ghost obstacles right under it.
  if (footprint_clearing && have_robot_pose) {
    clearFootprintRegion(*obstacle_layer, robot_x, robot_y, robot_yaw);
  }
  prev.points = std::move(world);
  prev.valid = true;
  wrapper->updateMapUnlocked(robot_x, robot_y, robot_yaw);
  wrapper->setCurrent(true);
  return true;
}

void CostmapNavigationServer::processObservation(
    const PendingObservation& observation, double* tf_wait_ms,
    bool* stamp_fallback) {
  if (stamp_fallback) {
    *stamp_fallback = false;
  }
  if (tf_wait_ms) {
    *tf_wait_ms = 0.0;
  }
  const auto* source = observation.source;
  if (source == nullptr || !costmap_context ||
      !costmap_context->localCostmap() ||
      (local_obstacle_layer == nullptr && local_layer_plugins.empty())) {
    spdlog::warn(
        "CostmapNavigationServer::processObservation: costmap or "
        "observation source not configured");
    return;
  }
  if (observation.points.empty() && observation.clearing_points.empty()) {
    return;
  }

  // Local map first (the controller's timescale; its diagnostics feed the
  // writer-loop timing).
  double local_tf_wait_ms = 0.0;
  bool local_stamp_fallback = false;
  const bool local_ok = stampObservationOnMap(
      observation, costmap_context->localFrame(),
      costmap_context->localCostmap(), local_obstacle_layer, prev_local,
      local_inflation_radius, /*is_local_map=*/true, &local_tf_wait_ms,
      &local_stamp_fallback, local_layer_plugins);
  if (tf_wait_ms) {
    *tf_wait_ms = local_tf_wait_ms;
  }
  if (stamp_fallback) {
    *stamp_fallback = local_stamp_fallback;
  }

  // Global map: the same observation is stamped into the global dynamic
  // layer when the source is listed under costmap_global.observation_sources
  // (nav2 global_costmap = static + obstacle + inflation), so replanning
  // and IsPathValid see real obstacles beyond the local rolling window.
  const bool global_active =
      !global_layer_plugins.empty() ||
      (global_obstacle_layer != nullptr &&
       std::find(global_observation_sources.begin(),
                 global_observation_sources.end(),
                 source->name) != global_observation_sources.end());
  if (global_active && costmap_context->globalCostmap()) {
    stampObservationOnMap(observation, costmap_context->globalFrame(),
                          costmap_context->globalCostmap(),
                          global_obstacle_layer, prev_global,
                          global_inflation_radius, /*is_local_map=*/false,
                          nullptr, nullptr, global_layer_plugins);
  }

  if (!local_ok) {
    return;
  }
  // Fresh data just applied: the map is current again (a configured
  // expected_update_rate marks it stale from the writer's liveness check).
  for (std::size_t i = 0; i < observation_sources.size(); ++i) {
    if (&observation_sources[i] == source) {
      source_last_seen[i] = std::chrono::steady_clock::now();
      break;
    }
  }

  // Scan-stamp query fallback rate observation (a fallback means that
  // frame's sensor transform degraded to latest-time semantics): a
  // persistently high fallback rate indicates the scan stamp deviates from
  // the TF sync chain beyond its cache window.
  if (stamp_query_fallback.load() > 0) {
    const auto diag_now = std::chrono::steady_clock::now();
    if (diag_now - last_stamp_diag_at >= std::chrono::seconds(10)) {
      last_stamp_diag_at = diag_now;
      spdlog::info("processObservation: stamp-queries ok={} fallback={}",
                   stamp_query_ok.load(), stamp_query_fallback.load());
      stamp_query_ok.store(0);
      stamp_query_fallback.store(0);
    }
  }
}

// ---- Rolling-window updates (writer thread when running) ----

void CostmapNavigationServer::updateRollingWindows() {
  // The resident map-writer thread owns rolling while it runs (single
  // literal writer); direct callers only roll in the inline/no-writer mode.
  if (writer_running.load()) {
    return;
  }
  rollWindows();
}

void CostmapNavigationServer::rollWindows() {
  if (!costmap_context) {
    return;
  }
  auto& robot_state_controller =
      ayanami::components::RobotStateController::instance();
  const auto now = std::chrono::steady_clock::now();

  // Local map: when rolling, center the window on the latest pose in the
  // LOCAL costmap frame (local_costmap_frame, typically odom). Pose
  // resolution happens before taking the lock; on query failure, reuse the
  // last successful pose (never fall back to the origin).
  if (local_roll_period.count() > 0.0 && costmap_context->localCostmap() &&
      costmap_context->localCostmap()->isRollingWindow() &&
      now - last_local_roll >= local_roll_period) {
    last_local_roll = now;
    const std::string& local_frame = costmap_context->localFrame();
    Eigen::Isometry3d t_robot = Eigen::Isometry3d::Identity();
    if (robot_state_controller.lookupTransform(
            local_frame, costmap_context->robotFrame(),
            ayanami::Timestamp::latest(), t_robot, k_tf_lookup_timeout_s)) {
      has_last_local_robot = true;
      last_local_robot_x = t_robot.translation().x();
      last_local_robot_y = t_robot.translation().y();
      last_local_robot_yaw =
          std::atan2(t_robot.linear()(1, 0), t_robot.linear()(0, 0));
    }
    if (has_last_local_robot) {
      auto lock = costmap_context->localCostmap()->lockForWrite();
      costmap_context->localCostmap()->updateMapUnlocked(
          last_local_robot_x, last_local_robot_y, last_local_robot_yaw);
    }
  }

  // Global map: only needed for rolling configs (production uses a static
  // map, skipped). Centered on the latest pose in the global costmap frame.
  if (global_roll_period.count() > 0.0 && costmap_context->globalCostmap() &&
      costmap_context->globalCostmap()->isRollingWindow() &&
      now - last_global_roll >= global_roll_period) {
    last_global_roll = now;
    const std::string& global_frame = costmap_context->globalFrame();
    Eigen::Isometry3d t_robot = Eigen::Isometry3d::Identity();
    if (robot_state_controller.lookupTransform(
            global_frame, costmap_context->robotFrame(),
            ayanami::Timestamp::latest(), t_robot, k_tf_lookup_timeout_s)) {
      const double gx = t_robot.translation().x();
      const double gy = t_robot.translation().y();
      const double gyaw =
          std::atan2(t_robot.linear()(1, 0), t_robot.linear()(0, 0));
      auto lock = costmap_context->globalCostmap()->lockForWrite();
      costmap_context->globalCostmap()->updateMapUnlocked(gx, gy, gyaw);
    }
  }
}

// ---- Map change command execution (sole writer) ----

bool CostmapNavigationServer::executeClearCommand(
    const ClearCostmapCommand& command) {
  if (!command.enabled) {
    return true;
  }
  if (!costmap_context) {
    spdlog::warn("CostmapNavigationServer::executeClearCommand: costmap not "
                 "configured");
    return false;
  }
  auto& robot_state_controller =
      ayanami::components::RobotStateController::instance();
  const std::string& global_frame = costmap_context->globalFrame();
  const std::string& local_frame = costmap_context->localFrame();

  const bool clear_global =
      command.affected_maps == "global" || command.affected_maps == "both";
  const bool clear_local =
      command.affected_maps == "local" || command.affected_maps == "both";
  const double radius =
      command.clear_radius > 0.0 ? command.clear_radius : 0.0;

  auto clearOne = [&](const costmap_2d::CostmapWrapper::Ptr& wrapper,
                       const std::string& frame) -> bool {
    if (!wrapper || frame.empty()) {
      return false;
    }
    Eigen::Isometry3d robot_pose = Eigen::Isometry3d::Identity();
    if (!robot_state_controller.lookupTransform(
            frame, costmap_context->robotFrame(),
            ayanami::Timestamp::latest(), robot_pose)) {
      return false;
    }
    // Sole writer: clear the dynamic layer + recompute + updateMap. The
    // mutable layers/master grid are never exposed to recovery modules.
    return clearCostmapArea(wrapper, robot_pose, radius);
  };

  bool ok = true;
  if (clear_global) {
    ok = clearOne(costmap_context->globalCostmap(), global_frame) && ok;
  }
  if (clear_local) {
    ok = clearOne(costmap_context->localCostmap(), local_frame) && ok;
  }
  return ok;
}

}  // namespace costmap
}  // namespace ayanami
