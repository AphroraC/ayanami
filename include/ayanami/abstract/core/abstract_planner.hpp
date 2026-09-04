#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 全局路径规划插件抽象 (MBF 式分层)。
 *
 * 零地图类型: 接口不包含任何地图参数。地图依赖只出现在地图族层
 * (如 CostmapPlanner) 的 configure(PluginContext, 地图模块) 注入点, 由宿主
 * (XXXNavigationServer) 持有地图模块所有权并注入。
 *
 * 职责表: 插件只做路径搜索; 失败立即以 ErrorCode 返回, 重试由 BT 决定。
 */
class AbstractPlanner : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractPlanner>;

  virtual ~AbstractPlanner() = default;

  /// @brief 计算 start→goal 的全局路径 (纯虚)。
  /// @return PlannerResultT: ok=true 携带路径; 失败携带 ErrorCode + 诊断。
  virtual PlannerResultT makePlan(
      const ayanami::geometry_msgs::PoseStamped& start,
      const ayanami::geometry_msgs::PoseStamped& goal,
      double tolerance) = 0;

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief 取消进行中的规划 (协作式中断; 默认不支持)。
  virtual bool cancel() { return false; }

  /// @brief 运行中 in-place 目标更新 (latest-wins seam; 可选)。
  virtual bool setNewGoal(const ayanami::geometry_msgs::PoseStamped&,
                          const ayanami::geometry_msgs::PoseStamped&, double) {
    return false;
  }
};

} // namespace abstract
} // namespace ayanami
