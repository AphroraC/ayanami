#pragma once

#include <atomic>
#include <memory>
#include <shared_mutex>
#include <string>
#include <utility>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>

namespace costmap_2d {

/**
 * @brief CostmapWrapper manages the LayeredCostmap lifecycle (active state) and
 * the read/write mutual exclusion of map data.
 *
 * Concurrency model (single writer + multiple readers, direct locked reads, zero copy):
 * - Writer (unique: CostmapNavigationServer / equivalent test writer): holds the unique lock to update
 *   layers (markObstacles/raytrace/clearArea) and calls updateMapUnlocked() to recompute
 *   the master grid. Map updates are infrequent (~10 Hz) and a single reader pass takes <10 ms,
 *   so the writer simply waits for readers to release.
 * - Readers (planner/smoother/controller/recovery plugins and visualization): take a
 *   shared lock via CostmapMapContext::lockGlobal()/lockLocal()/readGlobal()/readLocal() and read the
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

  /// @brief Writer path: takes the unique lock itself and updates the layers. Writers that need to combine
  /// this with other layer changes (e.g. clearCostmapArea) should hold the lock themselves and call updateMapUnlocked.
  void updateMap(double robot_x, double robot_y, double robot_yaw) {
    std::unique_lock<std::shared_mutex> lock(map_mutex);
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

  const std::string& getGlobalFrame() const { return global_frame; }

  bool isRollingWindow() const {
    return layered_costmap ? layered_costmap->isRolling() : false;
  }

 private:
  LayeredCostmap::Ptr layered_costmap;
  mutable std::atomic<bool> active{false};
  std::atomic<bool> current{true};
  mutable std::shared_mutex map_mutex;
  std::string global_frame;
};

}  // namespace costmap_2d
