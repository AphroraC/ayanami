#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 清代价地图恢复模块 (spec §6.6 costmap_clear_costmap_recovery)。
 *
 * 纯 C++ (零 ROS): 清除 global/local (按 affected_maps) master grid 中
 * robot 附近 radius 范围内的 cell → FREE_SPACE(0)。经 CostmapContext 直接
 * 改 master grid (clearArea 语义), 不需要底盘。
 */
class CostmapClearCostmapRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapClearCostmapRecovery();
  // Ruling #3: 基类 AbstractRecovery 无虚析构, 派生层补裸 virtual ~。
  virtual ~CostmapClearCostmapRecovery() = default;

  void initialize(const std::string& name,
                  ayanami::costmap::CostmapContext::Ptr ctx) override;

  uint32_t runRecovery(std::string& message) override;

  bool cancel() override;

 private:
  struct Config {
    double clear_radius = 1.0;
    std::string affected_maps = "both";  // global / local / both
  };

  void loadConfig();
  void clearCostmap(costmap_2d::Costmap2D* grid, double robot_x,
                    double robot_y) const;

  ayanami::costmap::CostmapContext::Ptr costmap_context_;
  std::atomic<bool> canceled_{false};
  Config config_;
};

}  // namespace modules
}  // namespace ayanami
