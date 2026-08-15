#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <costmap_2d/layered_costmap.hpp>

namespace costmap_2d {

// CostmapWrapper 管理 LayeredCostmap 的生命周期 (active 状态) 与线程安全互斥。
// - checkActivate / checkDeactivate 幂等: 仅在状态翻转 (false->true / true->false)
//   时返回 true 并 act, 重复调用无副作用。
// - updateMap 在 recursive_mutex 保护下驱动 rolling window 平移 + 各层刷新;
//   外部可通过 mutex() 在操作前持锁 (可与内部锁递归叠加)。
class CostmapWrapper {
 public:
  using Ptr = std::shared_ptr<CostmapWrapper>;

  explicit CostmapWrapper(LayeredCostmap::Ptr lm, std::string frame)
      : layered_costmap_(std::move(lm)), global_frame_(std::move(frame)) {}

  // 幂等激活: inactive->active 翻转返回 true, 重复调用返回 false 不重复 act。
  bool checkActivate() {
    bool expected = false;
    return active_.compare_exchange_strong(expected, true);
  }

  // 对偶: active->inactive 翻转返回 true, 重复调用返回 false 不重复 act。
  bool checkDeactivate() {
    bool expected = true;
    return active_.compare_exchange_strong(expected, false);
  }

  bool isActive() const { return active_.load(); }

  bool isCurrent() const { return current_.load(); }
  void setCurrent(bool current) { current_.store(current); }

  std::recursive_mutex& mutex() { return mutex_; }

  void updateMap(double robot_x, double robot_y, double robot_yaw) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (layered_costmap_) {
      layered_costmap_->updateMap(robot_x, robot_y, robot_yaw);
    }
  }

  LayeredCostmap& get() { return *layered_costmap_; }

  const std::string& getGlobalFrame() const { return global_frame_; }

  bool isRollingWindow() const {
    return layered_costmap_ ? layered_costmap_->isRolling() : false;
  }

 private:
  LayeredCostmap::Ptr layered_costmap_;
  std::atomic<bool> active_{false};
  std::atomic<bool> current_{true};
  std::recursive_mutex mutex_;
  std::string global_frame_;
};

}  // namespace costmap_2d
