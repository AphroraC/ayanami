#pragma once

#include "ayanami/abstract/core/abstract_goal_checker.hpp"
#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

  /**
   * @brief 局部控制器插件抽象 (spec §5.1, MBF 式分层)。
   *
   * 输入当前位姿/速度, 输出 cmd_vel; 接收待执行路径。到达判定由 execution
   * 层经 goal_checker 完成 (控制器自身不再判断是否到达目标)。
   * 零地图类型: 接口不包含任何地图参数; 地图依赖只出现在地图族层
   * (如 CostmapController) 的 initialize(地图模块) 注入点。
   */
class AbstractController : public virtual ExtensionModule {
public:
  using Ptr = std::shared_ptr<AbstractController>;

  virtual ~AbstractController() = default;

  /**
   * @brief 计算速度命令 (纯虚, 插件必须实现)。
   * @param[out] cmd_vel  体坐标系速度命令。
   * @param[out] message  失败时的人类可读诊断 (仅诊断, 决策只看 failure)。
   * @return FailureCode: None=成功, 其余为统一失败码。
   */
  virtual FailureCode computeVelocityCommands(
       const ayanami::geometry_msgs::PoseStamped& pose,
       const ayanami::geometry_msgs::TwistStamped& velocity,
       ayanami::geometry_msgs::TwistStamped& cmd_vel,
       std::string& message) = 0;

  /// @brief 设置要跟踪的路径。
  virtual bool setPlan(const ayanami::nav_msgs::Path& plan) = 0;

  virtual void configure() {}
  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief 取消进行中的控制。默认不支持, 返回 false。
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
