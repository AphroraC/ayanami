#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/core/costmap_recovery.hpp>

namespace ayanami {
namespace modules {

using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapCommandPort;

/**
 * @brief 清代价地图恢复模块 (spec §6.6 costmap_clear_costmap_recovery)。
 *
 * 纯 C++ (零 ROS): 周期驱动接口 (start/onCycleUpdate)。首周期构造类型化
 * ClearCostmapCommand 并经 CostmapCommandPort 提交给唯一地图写者
 * (CostmapNavigationServer) 执行, 随后 SUCCESS (无运动, 零速度)。插件不自行
 * sleep、不接触任何可变 wrapper / layer。
 * global 的静态底图由 StaticLayer 恢复, 不直接把 master grid 墙体写成 FREE。
 */
class CostmapClearCostmapRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  using ayanami::costmap::CostmapRecovery::runRecovery;
  CostmapClearCostmapRecovery();
  // Ruling #3: 基类 AbstractRecovery 无虚析构, 派生层补裸 virtual ~。
  virtual ~CostmapClearCostmapRecovery() = default;

  void initialize(const std::string& name,
                  ayanami::costmap::CostmapContext::ConstPtr map) override;

  /// @brief 注入写者命令端口 (地图变更只能经此提交)。
  void setCommandPort(CostmapCommandPort::Ptr port) override;

  FailureCode runRecovery(std::string& message) override;

  void start() override;
  ayanami::abstract::RecoveryCycleResult onCycleUpdate(
      std::string& message) override;
  void end() override;

  bool cancel() override;

 private:
  struct CostmapClearCostmapRecoveryParam {
    double clear_radius = 1.0;
    std::string affected_maps = "both";  // global / local / both
  };

  CostmapClearCostmapRecoveryParam param;

  std::string name;
  CostmapCommandPort::Ptr command_port;
  std::atomic<bool> canceled{false};
  bool submitted{false};
};

}  // namespace modules
}  // namespace ayanami
