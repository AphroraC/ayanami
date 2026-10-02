#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <string>
#include <utility>

#include <spdlog/spdlog.h>

#include <costmap/costmap_2d.hpp>
#include <costmap/layered_costmap.hpp>

namespace costmap_2d {

/**
 * @brief CostmapWrapper manages the LayeredCostmap lifecycle (active state) and
 * the read/write mutual exclusion of map data.
 *
 * "Current" is three distinct concepts; this class owns the first and
 * aggregates the second:
 * 1. sensor freshness -- the wrapper's atomic sensor_current, maintained by
 *    the sole map writer (observation source liveness: a silent source marks
 *    the map stale, fresh data re-marks it current);
 * 2. layer freshness -- costmap_2d::Layer::data_current, true once every
 *    input a layer received has been written through updateCosts;
 * 3. isCurrent() = sensor freshness AND all enabled layers current -- the
 *    single flag the execution safetyCheck gates controller/recovery loops
 *    on (stale sensor or unwritten input -> MAP_STALE -> BT recovery).
 *
 * Concurrency model (single writer + multiple readers, direct locked reads, zero copy):
 * - Writer (unique: CostmapNavigationServer / equivalent test writer): takes the unique lock via
 *   lockForWrite() to update layers (markObstacles/raytrace/clearArea) and calls updateMapUnlocked()
 *   to recompute the master grid. Map updates are infrequent (~10 Hz) and a reader pass takes
 *   <10 ms, so the writer simply waits for readers to release; lockForWrite() measures that wait
 *   and emits a throttled warning when it exceeds the threshold, so a misbehaving reader that
 *   starves the map writer becomes observable instead of silently stalling map updates.
 * - Readers (planner/smoother/controller/recovery modules and visualization): take a
 *   shared lock via CostmapMapContext::readGlobal()/readLocal() and read the
 *   master grid directly, with no copy and no snapshot.
 * - isCurrent()/setCurrent(): independent of the lock (atomic bool), used by execution
 *   safetyCheck.
 */
class CostmapWrapper {
 public:
  using Ptr = std::shared_ptr<CostmapWrapper>;

  explicit CostmapWrapper(LayeredCostmap::Ptr lm, std::string frame)
      : layered_costmap(std::move(lm)), global_frame(std::move(frame)) {}

  /// Idempotent activation: inactive->active flip returns true; repeated calls return false without re-acting.
  bool checkActivate() {
    bool expected = false;
    return active.compare_exchange_strong(expected, true);
  }
  bool checkActivate() const {
    bool expected = false;
    return active.compare_exchange_strong(expected, true);
  }

  /// Dual: active->inactive flip returns true; repeated calls return false without re-acting.
  bool checkDeactivate() {
    bool expected = true;
    return active.compare_exchange_strong(expected, false);
  }
  bool checkDeactivate() const {
    bool expected = true;
    return active.compare_exchange_strong(expected, false);
  }

  bool isActive() const { return active.load(); }

  /// @brief The isCurrent flag of (3) above: sensor freshness AND all
  /// enabled layers current.
  bool isCurrent() const {
    return sensor_current.load() &&
           (!layered_costmap || layered_costmap->isCurrent());
  }
  /// @brief Set the sensor-freshness half of isCurrent (sole map writer).
  void setCurrent(bool current) { this->sensor_current.store(current); }

  /// @brief Read/write lock: readers use shared_lock, the writer uses unique_lock (single writer only).
  std::shared_mutex& mutex() { return map_mutex; }
  std::shared_mutex& mutex() const { return map_mutex; }

  /// @brief Writer path with reader-starvation observability: acquires the
  /// unique lock and warns (throttled) when the acquisition waited longer
  /// than writer_wait_warn_s -- under the "<10ms per reader call" holding
  /// discipline a longer wait means some reader is overstaying.
  std::unique_lock<std::shared_mutex> lockForWrite() {
    const auto wait_start = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lock(map_mutex);
    const auto waited =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      wait_start);
    if (writer_wait_warn_s > 0.0 && waited.count() > writer_wait_warn_s) {
      constexpr std::int64_t k_warn_interval_ns = 5000000000LL;  // 5 s
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count();
      auto last = last_writer_warn_at.load();
      if (now_ns - last >= k_warn_interval_ns &&
          last_writer_warn_at.compare_exchange_strong(last, now_ns)) {
        // Route via the host's "costmap" layer logger when registered;
        // this sub-library depends on spdlog only.
        std::shared_ptr<spdlog::logger> costmap_logger = spdlog::get("costmap");
        if (!costmap_logger) {
          costmap_logger = spdlog::default_logger();
        }
        costmap_logger->warn(
          "CostmapWrapper[{}]: writer blocked on readers for {:.1f} ms "
          "(threshold {:.0f} ms); a reader is holding the shared lock too "
          "long and map updates are being delayed",
          global_frame,
          waited.count() * 1e3,
          writer_wait_warn_s * 1e3);
      }
    }
    return lock;
  }

  /// @brief Reader-wait warning threshold in seconds (<=0 disables).
  void setWriterWaitWarn(double seconds) { writer_wait_warn_s = seconds; }
  double getWriterWaitWarn() const { return writer_wait_warn_s; }

  /// @brief Writer path: takes the unique lock itself and updates the layers. Writers that need to combine
  /// this with other layer changes (e.g. clearCostmapArea) should hold the lock themselves and call updateMapUnlocked.
  void updateMap(double robot_x, double robot_y, double robot_yaw) {
    auto lock = lockForWrite();
    updateMapUnlocked(robot_x, robot_y, robot_yaw);
  }

  /// @brief Writer path (caller already holds the unique lock): updates layers and recomputes the master grid.
  /// Bumps the generation counter whenever the master grid actually changed.
  void updateMapUnlocked(double robot_x, double robot_y, double robot_yaw) {
    if (layered_costmap &&
        layered_costmap->updateMap(robot_x, robot_y, robot_yaw)) {
      master_generation.fetch_add(1, std::memory_order_release);
    }
  }

  /// @brief Monotonic master-grid version: bumped every time an update really
  /// mutated the grid (rolling origin shift or non-empty dirty-window rebuild;
  /// clear/resize paths all funnel through updateMapUnlocked). Readers compare
  /// generations to skip redundant copies -- no change means byte-identical.
  std::uint64_t generation() const {
    return master_generation.load(std::memory_order_acquire);
  }

  LayeredCostmap& get() { return *layered_costmap; }
  const LayeredCostmap& get() const { return *layered_costmap; }

  const std::string& globalFrame() const { return global_frame; }

  bool isRollingWindow() const {
    return layered_costmap ? layered_costmap->isRolling() : false;
  }

 private:
  LayeredCostmap::Ptr layered_costmap;
  mutable std::atomic<bool> active{false};
  /// @brief Sensor freshness (concept 1); layers aggregate their own
  /// data_current under LayeredCostmap::isCurrent.
  std::atomic<bool> sensor_current{true};
  /// @brief Master-grid mutation counter (see generation()).
  std::atomic<std::uint64_t> master_generation{0};
  mutable std::shared_mutex map_mutex;
  std::string global_frame;
  /// Reader-wait warning threshold (default 100 ms; readers are supposed to
  /// hold the lock for <10 ms per call) + 5 s warn throttle state. The
  /// throttle stamp is steady_clock nanoseconds in atomic<int64_t> (epoch 0
  /// = never warned): the primary std::atomic<time_point> deletes its
  /// default constructor on older libstdc++ (pre-C++20).
  double writer_wait_warn_s{0.1};
  std::atomic<std::int64_t> last_writer_warn_at{0};
};

}  // namespace costmap_2d
