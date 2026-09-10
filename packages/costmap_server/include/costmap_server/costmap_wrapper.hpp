#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <shared_mutex>
#include <string>
#include <utility>

#include <spdlog/spdlog.h>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>

namespace costmap_2d {

/**
 * @brief CostmapWrapper manages the LayeredCostmap lifecycle (active state) and
 * the read/write mutual exclusion of map data.
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

  bool isCurrent() const {
    return current.load() &&
           (!layered_costmap || layered_costmap->isCurrent());
  }
  void setCurrent(bool current) { this->current.store(current); }

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
      const auto now = std::chrono::steady_clock::now();
      auto last = last_writer_warn_at.load();
      if (now - last >= std::chrono::seconds(5) &&
          last_writer_warn_at.compare_exchange_strong(last, now)) {
        spdlog::warn(
            "CostmapWrapper[{}]: writer blocked on readers for {:.1f} ms "
            "(threshold {:.0f} ms); a reader is holding the shared lock too "
            "long and map updates are being delayed",
            global_frame, waited.count() * 1e3, writer_wait_warn_s * 1e3);
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
  void updateMapUnlocked(double robot_x, double robot_y, double robot_yaw) {
    if (layered_costmap) {
      layered_costmap->updateMap(robot_x, robot_y, robot_yaw);
    }
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
  std::atomic<bool> current{true};
  mutable std::shared_mutex map_mutex;
  std::string global_frame;
  /// Reader-wait warning threshold (default 100 ms; readers are supposed to
  /// hold the lock for <10 ms per call) + 5 s warn throttle state.
  double writer_wait_warn_s{0.1};
  std::atomic<std::chrono::steady_clock::time_point> last_writer_warn_at{
      std::chrono::steady_clock::time_point{}};
};

}  // namespace costmap_2d
