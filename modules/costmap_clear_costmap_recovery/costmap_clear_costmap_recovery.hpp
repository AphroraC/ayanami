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
 * 纯 C++ (零 ROS): 在各地图自己的 frame 中清除机器人附近的动态障碍层。
 * global 的静态底图由 StaticLayer 恢复，不直接把 master grid 墙体写成 FREE。
 */
class CostmapClearCostmapRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapClearCostmapRecovery();
  // Ruling #3: 基类 AbstractRecovery 无虚析构, 派生层补裸 virtual ~。
  virtual ~CostmapClearCostmapRecovery() = default;

  void initialize(const std::string& name,
                  ayanami::costmap::CostmapContext::Ptr ctx) override;

  FailureCode runRecovery(std::string& message) override;

  bool cancel() override;

 private:
  struct Config {
    double clear_radius = 1.0;
    std::string affected_maps = "both";  // global / local / both
  };

  void loadConfig();
  bool clearCostmap(const costmap_2d::CostmapWrapper::Ptr& wrapper,
                    const std::string& frame) const;

  ayanami::costmap::CostmapContext::Ptr costmap_context_;
  std::atomic<bool> canceled_{false};
  Config config_;
};

}  // namespace modules
}  // namespace ayanami
