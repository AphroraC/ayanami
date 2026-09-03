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
 * @brief CostmapWrapper 管理 LayeredCostmap 的生命周期 (active 状态) 与
 * 地图数据的读写互斥。
 *
 * 并发模型 (单写者 + 多读者, 直接加锁读, 零拷贝):
 * - 写者 (唯一: CostmapNavigationServer / 等价测试写者): 持 unique 锁更新
 *   层 (markObstacles/raytrace/clearArea) 并调 updateMapUnlocked() 重算
 *   master grid。地图更新频率低 (≈10Hz), 读者单次读图 <10ms, 写者等待
 *   读者释放即可。
 * - 读者 (planner/smoother/controller/recovery 插件及可视化): 经
 *   CostmapContext::lockGlobal()/lockLocal()/readGlobal()/readLocal() 取
 *   shared 锁直接读 master grid, 不拷贝、不快照。
 * - isCurrent()/setCurrent(): 与锁独立 (atomic bool), 供 execution
 *   safetyCheck 使用。
 */
class CostmapWrapper {
 public:
  using Ptr = std::shared_ptr<CostmapWrapper>;

  explicit CostmapWrapper(LayeredCostmap::Ptr lm, std::string frame)
      : layered_costmap(std::move(lm)), global_frame(std::move(frame)) {}

  // 幂等激活: inactive->active 翻转返回 true, 重复调用返回 false 不重复 act。
  bool checkActivate() {
    bool expected = false;
    return active.compare_exchange_strong(expected, true);
  }
  bool checkActivate() const {
    bool expected = false;
    return active.compare_exchange_strong(expected, true);
  }

  // 对偶: active->inactive 翻转返回 true, 重复调用返回 false 不重复 act。
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

  /// @brief 读写锁: 读者 shared_lock, 写者 unique_lock (仅单写者使用)。
  std::shared_mutex& mutex() { return map_mutex; }
  std::shared_mutex& mutex() const { return map_mutex; }

  /// @brief 写者路径: 自取 unique 锁更新层。需要与其它层变更组合的写者
  /// (如 clearCostmapArea) 应自行持锁后调 updateMapUnlocked()。
  void updateMap(double robot_x, double robot_y, double robot_yaw) {
    std::unique_lock<std::shared_mutex> lock(map_mutex);
    updateMapUnlocked(robot_x, robot_y, robot_yaw);
  }

  /// @brief 写者路径 (调用方已持 unique 锁): 更新层重算 master grid。
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
