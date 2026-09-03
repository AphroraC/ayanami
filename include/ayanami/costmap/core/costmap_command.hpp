#pragma once

#include <memory>
#include <string>

namespace ayanami {
namespace costmap {

/**
 * @brief 清代价地图命令 (typed clear command)。
 *
 * 由 CostmapCommandPort::submit() 交给唯一的地图写者
 * (CostmapNavigationServer) 执行: 写者在各目标 map 的 frame 内解析机器人
 * 位姿, 清空 clear_radius 内动态障碍层并重算 (updateMap → 发布新 epoch)。
 * 恢复插件本身不接触可变的 wrapper / layer。
 */
struct ClearCostmapCommand {
  std::string affected_maps = "both";  ///< "global" | "local" | "both"
  double clear_radius = 1.0;           ///< 以机器人为中心的清空半径 (m)。
  bool enabled = true;                 ///< false 视为 no-op (幂等跳过)。
};

/**
 * @brief 地图变更命令端口: 恢复/外部组件向写者提交类型化写命令的窄接口。
 *
 * 实现 (写者侧) 只有 CostmapNavigationServer 提供; 插件只持有该接口并
 * submit, 从而 "map mutation only through CostmapCommandPort"。
 */
class CostmapCommandPort {
 public:
  using Ptr = std::shared_ptr<CostmapCommandPort>;

  virtual ~CostmapCommandPort() = default;

  /// @brief 提交一条清代价地图命令。返回 true 表示写者已受理。
  virtual bool submit(const ClearCostmapCommand& command) = 0;
};

}  // namespace costmap
}  // namespace ayanami