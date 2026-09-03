#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 全局路径规划插件抽象 (spec §5.1, MBF 式分层)。
 *
 * 零地图类型: 接口不包含任何地图参数, 也不 include 任何地图头。地图依赖
 * 只出现在地图族层 (如 CostmapPlanner): 地图族基类新增
 * initialize(name, 地图模块 const 指针) 注入点, 由宿主 (XXXNavigation)
 * 持有地图模块所有权并注入。
 */
class AbstractPlanner : public virtual ExtensionModule {
public:
  using Ptr = std::shared_ptr<AbstractPlanner>;

  virtual ~AbstractPlanner() = default;

  /**
   * @brief 计算 start→goal 的全局路径 (纯虚, 插件必须实现)。
   * @param[out] plan  生成的路径。
   * @param[out] cost  路径代价。
   * @param[out] message 失败时的人类可读诊断 (仅诊断, 决策只看 failure)。
   * @return FailureCode: None=成功, 其余为统一失败码。
   */
  virtual FailureCode makePlan(const ayanami::geometry_msgs::PoseStamped& start,
                               const ayanami::geometry_msgs::PoseStamped& goal,
                               double tolerance, ayanami::nav_msgs::Path& plan,
                                double& cost, std::string& message) = 0;

  virtual void configure() {}
  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief 取消进行中的规划。默认不支持, 返回 false。
  virtual bool cancel() { return false; }

  virtual bool setNewGoal(const ayanami::geometry_msgs::PoseStamped&,
                          const ayanami::geometry_msgs::PoseStamped&, double) {
    return false;
  }
};

} // namespace abstract
} // namespace ayanami
