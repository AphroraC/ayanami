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

#include <ayanami/utility/bootstrap.hpp>
#include "ayanami/costmap/components/costmap_navigation_server.hpp"

#include <costmap/inflation_layer.hpp>
#include <costmap/simple_obstacle_layer.hpp>
#include <costmap/static_layer.hpp>
#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/costmap/components/costmap_commands.hpp"
#include <costmap/pgm_dump.hpp>
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
// Throttle stamps are steady_clock nanoseconds in std::atomic<int64_t>
// (epoch 0 = never warned): the primary std::atomic<time_point> deletes its
// default constructor on older libstdc++ (pre-C++20), which breaks builds
// there; integral atomics are portable across every toolchain.
bool tfWarnDue(std::atomic<std::int64_t>& last_warn_ns) {
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
  auto prev = last_warn_ns.load();
  if (now_ns - prev < static_cast<std::int64_t>(k_tf_warn_interval_s * 1e9)) {
    return false;
  }
  return last_warn_ns.compare_exchange_strong(prev, now_ns);
}

// Robot pose/rolling diagnostics throttling (one per 60s, for debugging; hints at a missing TF chain on failure).
bool robotDiagDue(std::atomic<std::int64_t>& last_diag_ns) {
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
  auto prev = last_diag_ns.load();
  if (now_ns - prev <
      static_cast<std::int64_t>(k_robot_diag_interval_s * 1e9)) {
    return false;
  }
  return last_diag_ns.compare_exchange_strong(prev, now_ns);
}

}  // namespace

CostmapNavigationServer::CostmapNavigationServer() = default;

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
      if (observation_buffer.empty()) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return observation_submitted.load() == observation_applied.load();
}

void CostmapNavigationServer::writerLoop() {
  while (writer_running.load()) {
    std::vector<PendingClear> clears;
    {
      std::unique_lock<std::mutex> lock(writer_mutex);
      writer_cv.wait_for(lock, std::chrono::milliseconds(10), [&] {
        return !observation_buffer.empty() || !pending_clears.empty() ||
               !writer_running.load();
      });
      if (!writer_running.load()) {
        break;
      }
      clears.swap(pending_clears);
    }

    // Clear commands first: a recovery's replan depends on the cleared map.
    for (auto& clear : clears) {
      const bool ok = executeClearCommand(clear.command);
      if (clear.done) {
        clear.done->set_value(ok);
      }
    }

    // One mark/raytrace + rebuild cycle per map at its configured
    // update_frequency (the batch is bounded by the buffer cap, so a burst
    // costs one rebuild, not one per frame). A global map that consumes no
    // observations and does not roll is fully static: its content only
    // changes through clear commands (executed above), so the periodic
    // cycle would be a guaranteed no-op and is skipped entirely.
    runMapCycle(/*is_local=*/true);
    if (globalMapNeedsCycle()) {
      runMapCycle(/*is_local=*/false);
    }

    // Sensor-liveness checks live on the writer thread too (single literal
    // writer for every map mutation).
    checkSourceLiveness();

    // Periodic field-diagnostics dump (costmap.dump_frequency Hz; the loop
    // wakes at ~100 Hz so the gate keeps the cadence).
    if (dump_period.count() > 0.0) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_dump >= dump_period) {
        last_dump = now;
        dumpCostmaps();
      }
    }
  }
}

void CostmapNavigationServer::runMapCycle(bool is_local, bool force) {
  if (!costmap_context) {
    return;
  }
  const auto& period = is_local ? local_roll_period : global_roll_period;
  auto& last_cycle = is_local ? last_local_cycle : last_global_cycle;
  // force = the inline (no-writer) path: the caller's thread runs the cycle
  // synchronously per submission, bypassing the update_frequency gate --
  // the legacy per-frame semantics tests rely on, through the SAME code
  // path the writer thread uses.
  const auto now = std::chrono::steady_clock::now();
  if (!force) {
    if (period.count() <= 0.0) {
      return;
    }
    if (now - last_cycle < period) {
      return;
    }
    last_cycle = now;
  }

  auto* wrapper =
      is_local ? costmap_context->localCostmap().get()
               : costmap_context->globalCostmap().get();
  if (!wrapper) {
    return;
  }
  // Static composition: only the local map consumes observations (the
  // global build is static+inflation and changes exclusively through clear
  // commands); a local map without sources has nothing to consume either.
  if (!is_local || !local_obstacle_applier || observation_sources.empty()) {
    return;
  }
  const std::string& map_frame =
      is_local ? costmap_context->localFrame() : costmap_context->globalFrame();

  struct ResolvedFrame {
    std::shared_ptr<const PendingObservation> frame;
    Eigen::Isometry3d t_frame_sensor{Eigen::Isometry3d::Identity()};
    bool transform_ok{false};
    double tf_wait_ms{0.0};
  };
  std::vector<ResolvedFrame> batch;

  // ---- take: consume this map's unread frames under the ingest mutex ----
  {
    std::lock_guard<std::mutex> lock(writer_mutex);
    auto& cursor = is_local ? local_cursor : global_cursor;
    while (cursor < observation_buffer.size()) {
      const auto& frame = observation_buffer[cursor];
      ++cursor;
      const double age_s = std::chrono::duration<double>(now - frame->received)
                               .count();
      if (observation_keep_time > 0.0 && age_s > observation_keep_time) {
        continue;  // aged out of the keep-time window: frame dropped
      }
      bool consumed = false;
      for (const auto& source : observation_sources) {
        if (source.kind == frame->source->kind) {
          consumed = true;
          break;
        }
      }
      if (!consumed) {
        continue;
      }
      batch.push_back(ResolvedFrame{frame});
    }
    // Storage compaction: drop the prefix both maps have walked past.
    const std::size_t min_cursor = std::min(local_cursor, global_cursor);
    if (min_cursor > 0) {
      observation_buffer.erase(observation_buffer.begin(),
                               observation_buffer.begin() +
                                   static_cast<std::ptrdiff_t>(min_cursor));
      local_cursor -= min_cursor;
      global_cursor -= min_cursor;
    }
    if (observation_buffer.empty()) {
      observation_applied.store(observation_submitted.load());
    }
  }

  // ---- resolve: stamp-time transforms + pose, never under a map lock ----
  for (auto& resolved : batch) {
    resolved.transform_ok = resolveObservationTransform(
        *resolved.frame, map_frame, is_local, resolved.t_frame_sensor,
        &resolved.tf_wait_ms);
  }
  // Static-zone mark filter setup (LOCAL map only): the local->global
  // transform is resolved HERE, outside any map lock (the transform source
  // must never be waited for under a map lock). The global grid is probed
  // per batch inside the cycle below (local write lock -> global read lock
  // is a safe order: the same thread is the sole writer of both maps).
  const bool filter_static =
      is_local && filter_static_marks && costmap_context &&
      static_cast<bool>(costmap_context->globalCostmap());
  Eigen::Isometry3d global_from_local = Eigen::Isometry3d::Identity();
  if (filter_static) {
    ayanami::components::RobotStateController::instance().lookupTransform(
        costmap_context->globalFrame(), costmap_context->localFrame(),
        ayanami::Timestamp::latest(), global_from_local, 0.0);
  }
  std::vector<Eigen::Vector3d> filtered_points;
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  const bool have_pose =
      resolveRobotPoseInMapFrame(map_frame, is_local, robot_x, robot_y,
                                 robot_yaw);

  // ---- apply + rebuild: one write lock, one updateMap for the batch ----
  bool any_applied = false;
  double tf_wait_total_ms = 0.0;
  const auto cycle_start = std::chrono::steady_clock::now();
  if (have_pose) {
    auto write_lock = wrapper->lockForWrite();
    for (auto& resolved : batch) {
      if (!resolved.transform_ok) {
        continue;
      }
      const auto& observation = *resolved.frame;
      ayanami::costmap::MapObservation map_observation;
      map_observation.t_frame_sensor = resolved.t_frame_sensor;
      const std::vector<Eigen::Vector3d>* points = &observation.points;
      if (filter_static) {
        // Drop hits whose global cell is already blocked: the static map
        // owns that geometry, and noise-marking it only makes the local
        // halo boundary flicker across legal close passes. Fail open:
        // an unusable global grid keeps every point.
        filtered_points.clear();
        filtered_points.reserve(observation.points.size());
        const auto global = costmap_context->globalCostmap();
        bool probe_usable = false;
        if (global) {
          std::shared_lock<std::shared_mutex> read_lock(global->mutex());
          const auto* grid = global->get().getCostmap();
          if (grid && grid->getSizeInCellsX() > 0) {
            probe_usable = true;
            for (const auto& p : observation.points) {
              const Eigen::Vector3d pg =
                  global_from_local * (resolved.t_frame_sensor * p);
              unsigned mx = 0;
              unsigned my = 0;
              if (!grid->worldToMap(pg.x(), pg.y(), mx, my)) {
                filtered_points.push_back(p);  // unmapped: the cloud is all we have
                continue;
              }
              if (grid->getCost(mx, my) >=
                  costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                continue;  // static zone: the global map already blocks it
              }
              filtered_points.push_back(p);
            }
          }
        }
        if (!probe_usable) {
          filtered_points = observation.points;
        }
        points = &filtered_points;
      }
      map_observation.points = points;
      map_observation.clearing_points = &observation.clearing_points;
      map_observation.robot_pose_xy_yaw =
          Eigen::Vector3d(robot_x, robot_y, robot_yaw);
      map_observation.has_robot_pose = have_pose;
      for (const auto& source : observation_sources) {
        if (source.kind != observation.source->kind) {
          continue;
        }
        map_observation.source = &source;
        local_obstacle_applier->apply(map_observation);
        break;
      }
      any_applied = true;
      tf_wait_total_ms += resolved.tf_wait_ms;
    }
    // Rolling recenter + dirty-window rebuild run every cycle, batch or not
    // (updateMap early-outs on an empty dirty window; the rolling origin
    // shift is whole-cell and cheap).
    wrapper->updateMapUnlocked(robot_x, robot_y, robot_yaw);
    if (any_applied) {
      wrapper->setCurrent(true);
    }
  }

  // Unconditional per-layer emission on real content updates (generation
  // gate makes repeat calls no-ops on unchanged maps).
  emitMapLayers(is_local);

  if (any_applied) {
    // Liveness: every consumed frame's source was seen.
    for (std::size_t i = 0; i < observation_sources.size(); ++i) {
      for (const auto& resolved : batch) {
        if (!resolved.transform_ok) {
          continue;
        }
        if (&observation_sources[i] == resolved.frame->source) {
          source_last_seen[i] = std::chrono::steady_clock::now();
        }
      }
    }
    // nav2 mapUpdateLoop-style cycle timing: the designed bounded TF wait
    // is excluded; only rasterization/inflation work is judged.
    const double cycle_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - cycle_start)
            .count();
    const double work_ms = cycle_ms - tf_wait_total_ms;
    if (work_ms > k_map_update_warn_ms) {
      const auto warn_now = std::chrono::steady_clock::now();
      if (warn_now - last_map_update_warn_at >= std::chrono::seconds(5)) {
        last_map_update_warn_at = warn_now;
        SPDLOG_LOGGER_WARN(
          ayanami::costmapLogger(),
          "CostmapNavigationServer: {} map cycle ({} frames) work took "
          "{:.1f} ms (threshold {:.0f} ms; total {:.1f} ms incl. {:.1f} "
          "ms stamp-time TF wait); rasterization/inflation is the "
          "bottleneck -- observation ingestion stays O(enqueue)",
          is_local ? "local" : "global",
          batch.size(),
          work_ms,
          k_map_update_warn_ms,
          cycle_ms,
          tf_wait_total_ms);
      }
    }

    // Stamp-query drop-rate observation (a drop means that frame's sensor
    // transform never arrived at stamp time): a persistently high drop rate
    // indicates the sensor stamp deviates from the TF chain beyond the sync
    // window.
    if (stamp_query_fallback.load() > 0) {
      const auto diag_now = std::chrono::steady_clock::now();
      if (diag_now - last_stamp_diag_at >= std::chrono::seconds(10)) {
        last_stamp_diag_at = diag_now;
        SPDLOG_LOGGER_INFO(
          ayanami::costmapLogger(),
          "runMapCycle({}): stamp-queries ok={} dropped={}",
          is_local ? "local" : "global",
          stamp_query_ok.load(),
          stamp_query_fallback.load());
        stamp_query_ok.store(0);
        stamp_query_fallback.store(0);
      }
    }
  }
}

// ---- Per-layer artifact emission (generation-gated) ----

namespace {

/// @brief Master-grid / layer-grid -> OccupancyGrid (the visualization
/// mapping the host's incremental publisher used: NO_INFORMATION -> -1,
/// lethal -> 100, otherwise linear over 0..252).
ayanami::nav_msgs::OccupancyGrid toOccupancyGrid(const costmap_2d::Costmap2D& cm,
                                                 const std::string& frame) {
  ayanami::nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = frame;
  grid.info.resolution = cm.getResolution();
  grid.info.width = cm.getSizeInCellsX();
  grid.info.height = cm.getSizeInCellsY();
  Eigen::Isometry3d origin_iso = Eigen::Isometry3d::Identity();
  origin_iso.translation() =
      Eigen::Vector3d(cm.getOriginX(), cm.getOriginY(), 0.0);
  grid.info.origin = ayanami::geometry_msgs::Pose::fromIsometry(origin_iso);
  const uint32_t width = grid.info.width;
  const uint32_t height = grid.info.height;
  grid.data.resize(static_cast<std::size_t>(width) * height);
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      const unsigned char cost = cm.getCost(x, y);
      std::int8_t occ;
      if (cost == costmap_2d::NO_INFORMATION) {
        occ = -1;
      } else if (costmap_2d::isLethal(cost)) {
        occ = 100;
      } else {
        occ = static_cast<std::int8_t>((static_cast<int>(cost) * 100) / 252);
      }
      grid.data[static_cast<std::size_t>(y) * width + x] = occ;
    }
  }
  return grid;
}

}  // namespace

void CostmapNavigationServer::emitMapLayers(bool is_local) {
  if (!costmap_context) {
    return;
  }
  const auto& wrapper =
      is_local ? costmap_context->localCostmap() : costmap_context->globalCostmap();
  if (!wrapper) {
    return;
  }
  const std::uint64_t gen = wrapper->generation();
  auto& ever = is_local ? local_layers_ever_emitted
                        : global_layers_ever_emitted;
  auto& emitted = is_local ? emitted_local_generation
                           : emitted_global_generation;
  if (ever.load(std::memory_order_relaxed) &&
      emitted.load(std::memory_order_relaxed) == gen) {
    return;  // unchanged since the last emission
  }
  const std::string& frame =
      is_local ? costmap_context->localFrame() : costmap_context->globalFrame();

  if (is_local) {
    // Two local artifacts: the obstacle layer's RAW marks (presence grid,
    // no inflation halo) and the inflated master. Both rendered under the
    // same shared lock; emitted with NO map lock held (slot receivers copy
    // freely).
    ayanami::nav_msgs::OccupancyGrid obstacle_grid;
    bool have_obstacle = false;
    ayanami::nav_msgs::OccupancyGrid grid;
    {
      std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
      const auto* cm = wrapper->get().getCostmap();
      if (!cm) {
        return;
      }
      if (local_obstacle_layer) {
        const auto* obstacle_layer = local_obstacle_layer.get();
        {
          const auto& presence = obstacle_layer->presenceGrid();
          const unsigned px = obstacle_layer->geomSizeX();
          const unsigned py = obstacle_layer->geomSizeY();
          obstacle_grid.header.frame_id = frame;
          obstacle_grid.info.resolution = obstacle_layer->geomResolution();
          obstacle_grid.info.width = px;
          obstacle_grid.info.height = py;
          Eigen::Isometry3d origin_iso = Eigen::Isometry3d::Identity();
          origin_iso.translation() = Eigen::Vector3d(
              obstacle_layer->geomOriginX(), obstacle_layer->geomOriginY(),
              0.0);
          obstacle_grid.info.origin =
              ayanami::geometry_msgs::Pose::fromIsometry(origin_iso);
          obstacle_grid.data.resize(static_cast<std::size_t>(px) * py);
          for (unsigned y = 0; y < py; ++y) {
            for (unsigned x = 0; x < px; ++x) {
              const unsigned char mark =
                  presence[static_cast<std::size_t>(y) * px + x];
              std::int8_t occ = 0;
              if (mark != 0) {
                occ = static_cast<std::int8_t>(
                    (static_cast<int>(mark) * 100) / 252);
                if (occ > 100) occ = 100;
                if (occ < 1) occ = 1;
              }
              obstacle_grid.data[static_cast<std::size_t>(y) * px + x] = occ;
            }
          }
          have_obstacle = true;
        }
      }
      grid = toOccupancyGrid(*cm, frame);
    }
    ever.store(true, std::memory_order_relaxed);
    emitted.store(gen, std::memory_order_relaxed);
    if (have_obstacle) {
      ayanami::interfaces::NavigationCallbacks::on_local_costmap_obstacle(
          obstacle_grid);
    }
    ayanami::interfaces::NavigationCallbacks::on_local_costmap_inflation(grid);
    return;
  }

  // Global: the static layer's own grid + the inflated master.
  ayanami::nav_msgs::OccupancyGrid static_grid;
  bool have_static = false;
  if (global_static_layer) {
    // Static content is configure-time only: safe to read lock-free.
    static_grid = toOccupancyGrid(global_static_layer->layerMap(), frame);
    have_static = true;
  }
  ayanami::nav_msgs::OccupancyGrid master_grid;
  {
    std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
    const auto* cm = wrapper->get().getCostmap();
    if (!cm) {
      return;
    }
    master_grid = toOccupancyGrid(*cm, frame);
  }
  ever.store(true, std::memory_order_relaxed);
  emitted.store(gen, std::memory_order_relaxed);
  if (have_static) {
    ayanami::interfaces::NavigationCallbacks::on_global_costmap_static(
        static_grid);
  }
  ayanami::interfaces::NavigationCallbacks::on_global_costmap_inflation(
      master_grid);
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
    if (now - prev >= std::chrono::seconds(5) || prev == std::chrono::steady_clock::time_point{}) {
      source_stale_warn_at[i] = now;
      SPDLOG_LOGGER_WARN(
        ayanami::costmapLogger(),
        "CostmapNavigationServer: observation source '{}' has been silent "
        "for {:.1f}s (expected every {:.1f}s); the costmap is marked "
        "not-current so the controller/execution safety checks degrade to "
        "recovery instead of trusting stale obstacle data",
        source.name,
        silent_s,
        source.expected_update_rate);
    }
  }
  if (!all_fresh) {
    costmap_context->localCostmap()->setCurrent(false);
    // A global obstacle layer consuming sensor streams degrades together
    // with the local one (same observations, same staleness). The guard is
    // "the global map actually consumes observations", NOT "it has layers":
    // a static global (static+inflation only) has no sensor input, so its
    // freshness must not depend on sensor liveness.
    if (costmap_context->globalCostmap() && global_consumes_observations) {
      costmap_context->globalCostmap()->setCurrent(false);
    }
  }
}

void CostmapNavigationServer::configureCoreMaps(const std::string& config_path) {
  // Double configureCoreMaps guard. When a built context already exists,
  // re-entry is ignored: rebuilding would replace costmap_context and leave
  // the wrapper references of already-injected modules dangling (after the
  // old context is replaced by the new object, modules still point at the
  // old wrapper).
  if (costmap_context && (costmap_context->globalCostmap() || costmap_context->localCostmap())) {
    SPDLOG_LOGGER_WARN(
      ayanami::costmapLogger(),
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
  // Static composition (fixed): StaticLayer + InflationLayer. The builder
  // also performs the former static plugin's onAddedToMap work (pgm load +
  // master sizing / fallback geometry).
  const CostmapLayerAssembly global_layer_assembly =
      buildGlobalLayers(cfg, *global_lm);
  global_static_layer = global_layer_assembly.static_layer;
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

  // Static composition (fixed): SimpleObstacleLayer + InflationLayer; the
  // observation sources and the applier come with the assembly.
  const CostmapLayerAssembly local_layer_assembly = buildLocalLayers(cfg);
  local_lm->addLayer(local_layer_assembly.obstacle_layer);
  local_lm->addLayer(local_layer_assembly.inflation_layer);
  local_obstacle_layer = local_layer_assembly.obstacle_layer;
  local_obstacle_applier = local_layer_assembly.obstacle_applier;

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

  // Observation keep-time window (nav2 observation_keep_time): frames older
  // than this are dropped at consumption time instead of being processed
  // late (a stale frame's sector is better re-covered by a fresh one).
  observation_keep_time =
      cfg.param<double>("costmap_local", "observation_keep_time", 0.5);
  // Static-zone mark filter: cloud hits landing in a cell the GLOBAL map
  // already blocks (cost >= INSCRIBED) are not marked into the local map --
  // the static map owns that geometry, and re-marking it from live range
  // noise made the local halo boundary flicker across paths that legally
  // pass ~0.3 m from walls (sustained IsPathValid FAIL churn).
  filter_static_marks =
      cfg.param<bool>("costmap_local", "filter_static_marks", true);

  // Observation sources = the local obstacle layer's parsed list (the
  // global build consumes none): the writer's kind lookup and the liveness
  // check see one list. Filtering parameters are layer-owned.
  observation_sources = local_layer_assembly.observation_sources;
  // Liveness baselines: the writer start re-baselines them, but the inline
  // (no-writer) processing path stamps them too, so size them here.
  source_last_seen.assign(observation_sources.size(),
                          std::chrono::steady_clock::now());
  source_stale_warn_at.assign(observation_sources.size(),
                              std::chrono::steady_clock::time_point{});
  // Whether the global cycle consumes observations at all: when it does
  // not, its cursor tracks the buffer tail (submitObservation) so the
  // shared storage compacts.
  // The global build is fixed static+inflation: it never consumes
  // observations (a special deployment loading a dynamic global layer no
  // longer exists -- the static-global policy is now the composition).
  global_consumes_observations = false;
  // Stamp-time TF bounded wait (writer-thread budget; publishers never
  // block). Footprint clearing is an obstacle-layer plugin parameter.
  transform_tolerance =
      cfg.param<double>("costmap_local", "transform_tolerance", 0.1);

  // Periodic costmap dump (field diagnostics): costmap.dump_frequency Hz
  // on the writer thread (0 = disabled; production runs at 0.1 Hz). The
  // dump lands in the logging section's logging_dir -- the same directory
  // the file logger uses, one place to collect a run's artifacts.
  const double dump_frequency =
      cfg.param<double>("costmap", "dump_frequency", 0.0);
  dump_period = dump_frequency > 0.0
                    ? std::chrono::duration<double>(1.0 / dump_frequency)
                    : std::chrono::duration<double>(0.0);
  if (ayanami::GlobalConfig::instance()) {
    dump_dir = ayanami::GlobalConfig::instance()->param<std::string>(
        "logging", "logging_dir", std::string("logs"));
  }
  {
    std::error_code ec;
    std::filesystem::create_directories(dump_dir, ec);
    if (ec) {
      SPDLOG_LOGGER_WARN(
        ayanami::costmapLogger(),
        "CostmapNavigationServer: cannot create dump dir '{}': "
        "{}; costmap dumps disabled",
        dump_dir,
        ec.message());
      dump_period = std::chrono::duration<double>(0.0);
    }
  }

  costmap_context = std::make_shared<CostmapMapContext>();
  costmap_context->setGlobalCostmap(global_wrapper);
  costmap_context->setLocalCostmap(local_wrapper);
  costmap_context->setFrames(global_frame, local_frame, robot_frame);
  // Writer-side reader-starvation observability (see CostmapWrapper).
  global_wrapper->setWriterWaitWarn(k_writer_wait_warn_s);
  local_wrapper->setWriterWaitWarn(k_writer_wait_warn_s);
  // Initial per-layer emission: a static global may never cycle again, so
  // this configure-time emission can be the ONLY one ever (subscribers
  // -- e.g. ayanami_ros costmap_viewer -- attach before navigation start).
  emitMapLayers(false);
  emitMapLayers(true);
  // The footprint is cached as an immutable copy: snapshot building reads
  // only this cache (never read the live LayeredCostmap's footprint outside
  // the write lock).
  costmap_context->setFootprint(footprint);

  // One-time configuration of the global RobotStateController: frame names
  // + velocity timeout are cached only here; the hot getRobotState never
  // reads Config (extension).
  const double velocity_timeout =
      nav_cfg.param<double>("navigation", "velocity_timeout", 0.25);
  ayanami::components::RobotStateController::instance().initialize(
      global_frame, local_frame, robot_frame, velocity_timeout);

  has_last_local_robot = false;
}


bool CostmapNavigationServer::initializePlannerModule(
  const std::string& name,
  const ayanami::Config& module_config,
  ayanami::abstract::AbstractPlanner::Ptr ptr) {
  if (!costmap_context) {
    SPDLOG_LOGGER_ERROR(
      ayanami::costmapLogger(),
      "CostmapNavigationServer::initializePlannerModule('{}'): costmap "
      "context not configured (call configureCoreMaps first)",
      name);
    return false;
  }
  auto cast = std::dynamic_pointer_cast<CostmapPlanner>(ptr);
  if (!cast) {
    SPDLOG_LOGGER_ERROR(
      ayanami::costmapLogger(),
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
  const std::string& name,
  const ayanami::Config& module_config,
  ayanami::abstract::AbstractSmoother::Ptr ptr) {
  if (!costmap_context) {
    SPDLOG_LOGGER_ERROR(
      ayanami::costmapLogger(),
      "CostmapNavigationServer::initializeSmootherModule('{}'): costmap "
      "context not configured (call configureCoreMaps first)",
      name);
    return false;
  }
  auto cast = std::dynamic_pointer_cast<CostmapSmoother>(ptr);
  if (!cast) {
    SPDLOG_LOGGER_ERROR(
      ayanami::costmapLogger(),
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
  const std::string& name,
  const ayanami::Config& module_config,
  ayanami::abstract::AbstractController::Ptr ptr) {
  if (!costmap_context) {
    SPDLOG_LOGGER_ERROR(
      ayanami::costmapLogger(),
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
    SPDLOG_LOGGER_ERROR(
      ayanami::costmapLogger(),
      "CostmapNavigationServer::initializeControllerModule('{}'): module is "
      "not a CostmapController (costmap family interface mismatch)",
      name);
    return false;
  }
  cast->configure(ayanami::ModuleContext{name, module_config},
                  costmap_context->getSharedConstPtr());
  return true;
}

// ---- Obstacle observation map stamping ----

void CostmapNavigationServer::handleObstaclePointCloud(const ayanami::sensor_msgs::PointCloud2& cloud) {
  if (!costmap_context || !costmap_context->localCostmap()) {
    SPDLOG_LOGGER_WARN(
      ayanami::costmapLogger(),
      "CostmapNavigationServer::handleObstaclePointCloud: costmap not "
      "configured (call configureCoreMaps first)");
    return;
  }
  const auto* source = findSource(ObservationSourceConfig::Kind::PointCloud);
  if (source == nullptr) {
    static std::atomic<bool> warned{false};
    if (!warned.exchange(true)) {
      SPDLOG_LOGGER_WARN(
        ayanami::costmapLogger(),
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
  // Every hit doubles as a freespace ray endpoint (nav2 combined
  // marking+clearing semantics for an obstacle cloud): the map cycle's
  // endpoint handling applies raytrace_min_range skipping and
  // raytrace_max_range clamping, exactly as for laser beams. Endpoints are
  // NOT height-gated: a ray is 2D free-space evidence along its path, the
  // min/max_obstacle_height window gates marks only.
  observation.clearing_points.reserve(cloud.cloud.points.size());
  for (const auto& p : cloud.cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    // z is KEPT here: the source's min/max_obstacle_height window filters
    // hits in the map cycle (nav2 semantics).
    observation.points.emplace_back(static_cast<double>(p.x),
                                    static_cast<double>(p.y),
                                    static_cast<double>(p.z));
    observation.clearing_points.emplace_back(static_cast<double>(p.x),
                                             static_cast<double>(p.y),
                                             static_cast<double>(p.z));
  }
  submitObservation(std::move(observation));
}

void CostmapNavigationServer::handleObstacleLaserScan(const ayanami::sensor_msgs::LaserScan& scan) {
  if (!costmap_context || !costmap_context->localCostmap()) {
    SPDLOG_LOGGER_WARN(
      ayanami::costmapLogger(),
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
      SPDLOG_LOGGER_WARN(
        ayanami::costmapLogger(),
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
    // blocked"); the map cycle skips rays below raytrace_min_range.
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
  auto frame =
      std::make_shared<const PendingObservation>(std::move(observation));
  {
    std::lock_guard<std::mutex> lock(writer_mutex);
    if (observation_buffer.size() >= k_max_pending_observations) {
      // Sustained processing stall: drop the OLDEST frame (its sector
      // coverage is the stalest) and keep the freshest ones. Counter +
      // throttled warning: sustained drops mean the writer cannot keep up
      // and must be grep-able in the field.
      observation_buffer.pop_front();
      observations_dropped.fetch_add(1);
      if (local_cursor > 0) {
        --local_cursor;
      }
      if (global_cursor > 0) {
        --global_cursor;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now - last_drop_warn_at >= std::chrono::seconds(5)) {
        last_drop_warn_at = now;
        SPDLOG_LOGGER_WARN(
          ayanami::costmapLogger(),
          "CostmapNavigationServer: observation buffer overflow -- "
          "{} frames dropped so far (cap {}); the map writer cannot "
          "keep up with the sensor rate",
          observations_dropped.load(),
          k_max_pending_observations);
      }
    }
    observation_buffer.push_back(std::move(frame));
    if (!global_consumes_observations) {
      // No global consumer: the global cursor tracks the tail so the
      // shared storage still compacts.
      global_cursor = observation_buffer.size();
    }
  }
  if (!writer_running.load()) {
    // No writer thread (direct-construction tests): the caller's thread
    // runs the SAME cycle pipeline synchronously per submission
    // (update_frequency gate bypassed) -- one implementation, two drivers.
    // The static-global skip matches the writer-thread path.
    runMapCycle(/*is_local=*/true, /*force=*/true);
    if (globalMapNeedsCycle()) {
      runMapCycle(/*is_local=*/false, /*force=*/true);
    }
    observation_applied.store(observation_submitted.load());
    return;
  }
  writer_cv.notify_all();
}

bool CostmapNavigationServer::globalMapNeedsCycle() const {
  // The periodic global cycle only does real work when the global map
  // consumes observations (dynamic obstacle layer) or rolls; a static
  // global (static+inflation, non-rolling) changes exclusively through
  // clear commands, which are executed separately on the writer path.
  if (global_consumes_observations) {
    return true;
  }
  return costmap_context && costmap_context->globalCostmap() &&
         costmap_context->globalCostmap()->isRollingWindow();
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

bool CostmapNavigationServer::resolveObservationTransform(
    const PendingObservation& observation, const std::string& map_frame,
    bool is_local_map, Eigen::Isometry3d& t_frame_sensor, double* tf_wait_ms) {
  (void)is_local_map;
  if (tf_wait_ms) {
    *tf_wait_ms = 0.0;
  }
  auto& robot_state_controller =
      ayanami::components::RobotStateController::instance();
  const bool have_stamp = !observation.stamp.isLatest();
  // A frame published WITHOUT a stamp is latest-time by construction -- no
  // stamp-time skew exists, so it resolves (and marks) at latest time. A
  // frame that CARRIED a stamp resolves at stamp time only: degrading it to
  // a latest-time lookup would bake omega x dt skew into the hits, and its
  // clearing rays end ON hits, so a degraded frame would erase the very
  // obstacles it just saw. Such frames are dropped whole -- existing marks
  // stay, the map goes stale-but-stable (nav2 drops the observation on TF
  // failure for the same reason).

  double lookup_wait_ms = 0.0;
  if (have_stamp) {
    if (lookupWithBoundedWait(map_frame, observation.frame_id,
                              observation.stamp, t_frame_sensor,
                              &lookup_wait_ms)) {
      ++stamp_query_ok;
      if (tf_wait_ms) {
        *tf_wait_ms = lookup_wait_ms;
      }
      return true;
    }
    ++stamp_query_fallback;
    if (tfWarnDue(map_tf_warn_at)) {
      SPDLOG_LOGGER_WARN(
        ayanami::costmapLogger(),
        "CostmapNavigationServer: stamp-time transform {} <- {} unavailable "
        "within transform_tolerance (stamp={:f}s); frame DROPPED, existing "
        "marks preserved. Persistently high drop rates mean the sensor "
        "stamp deviates from the TF chain beyond the sync window",
        map_frame,
        observation.frame_id,
        observation.stamp.toSeconds());
    }
    return false;
  }
  if (!robot_state_controller.lookupTransform(
          map_frame, observation.frame_id, ayanami::Timestamp::latest(),
          t_frame_sensor, k_tf_lookup_timeout_s)) {
    if (tfWarnDue(map_tf_warn_at)) {
      SPDLOG_LOGGER_WARN(
        ayanami::costmapLogger(),
        "CostmapNavigationServer: no transform {} <- {} (latest); "
        "obstacle stamping paused. Check the TF producers (gazebo "
        "diff-drive odom->base_footprint, robot_state_publisher) and "
        "whether a leftover simulation instance publishes duplicate TF",
        map_frame,
        observation.frame_id);
    }
    return false;
  }
  if (tf_wait_ms) {
    *tf_wait_ms = lookup_wait_ms;
  }
  return true;
}

bool CostmapNavigationServer::resolveRobotPoseInMapFrame(
    const std::string& map_frame, bool is_local_map, double& robot_x,
    double& robot_y, double& robot_yaw) {
  auto& robot_state_controller =
      ayanami::components::RobotStateController::instance();
  Eigen::Isometry3d t_robot = Eigen::Isometry3d::Identity();
  if (robot_state_controller.lookupTransform(
          map_frame, costmap_context->robotFrame(),
          ayanami::Timestamp::latest(), t_robot, k_tf_lookup_timeout_s)) {
    robot_x = t_robot.translation().x();
    robot_y = t_robot.translation().y();
    robot_yaw = std::atan2(t_robot.linear()(1, 0), t_robot.linear()(0, 0));
    if (is_local_map) {
      has_last_local_robot = true;
      last_local_robot_x = robot_x;
      last_local_robot_y = robot_y;
      last_local_robot_yaw = robot_yaw;
    }
    return true;
  }
  if (is_local_map && has_last_local_robot) {
    // Transient pose loss: roll/consume with the last successful pose --
    // never (0,0,0), which would anchor the rolling window back to the
    // frame origin and strand the controller outside the map.
    robot_x = last_local_robot_x;
    robot_y = last_local_robot_y;
    robot_yaw = last_local_robot_yaw;
    return true;
  }
  return false;
}

// ---- Periodic costmap dump (writer thread, costmap.dump_frequency) ----

void CostmapNavigationServer::dumpCostmaps() {
  if (!costmap_context) {
    return;
  }
  const auto dump_one = [&](bool is_local) {
    // Short-lock snapshot copy; the file I/O runs outside the map lock
    // (the same read discipline the planner/smoother snapshots use).
    ayanami::costmap::CostmapSnapshot snap;
    const bool ok = is_local ? costmap_context->snapshotLocal(snap)
                             : costmap_context->snapshotGlobal(snap);
    if (!ok || !snap.valid()) {
      return;
    }
    const std::string name = is_local ? "costmap_local" : "costmap_global";
    std::string error;
    if (!costmap_2d::writePgmDump(snap.costs.data(), snap.size_x, snap.size_y,
                                  snap.resolution, snap.origin_x, snap.origin_y,
                                  std::filesystem::path(dump_dir) / name,
                                  &error)) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_drop_warn_at >= std::chrono::seconds(5)) {
        last_drop_warn_at = now;
        SPDLOG_LOGGER_WARN(
          ayanami::costmapLogger(),
          "CostmapNavigationServer: costmap dump '{}' failed: {}",
          name,
          error);
      }
    }
  };
  dump_one(/*is_local=*/false);
  dump_one(/*is_local=*/true);
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
      now - last_local_cycle >= local_roll_period) {
    last_local_cycle = now;
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
      now - last_global_cycle >= global_roll_period) {
    last_global_cycle = now;
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
    SPDLOG_LOGGER_WARN(
      ayanami::costmapLogger(),
      "CostmapNavigationServer::executeClearCommand: costmap not "
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
    // A static global only changes through clears: re-emit its artifacts.
    emitMapLayers(false);
  }
  if (clear_local) {
    ok = clearOne(costmap_context->localCostmap(), local_frame) && ok;
    emitMapLayers(true);
  }
  return ok;
}

}  // namespace costmap
}  // namespace ayanami
